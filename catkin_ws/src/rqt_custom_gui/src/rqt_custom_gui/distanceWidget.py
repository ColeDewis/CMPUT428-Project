import os
import rospy
import rospkg

#from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
#from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtGui import QPixmap, QImage
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from std_msgs.msg import Empty
from custom_msgs.msg import TrackRequest, ErrorDefinition, Point2D, DistanceDefinition
import cv2

from time import sleep

class DistancePlace(QWidget):
    def __init__(self, imName, distanceDef,qLabel):
        super(DistancePlace, self).__init__()
        self.Distance = distanceDef
        self.Distance.plane_points = []
        self.clickCount = 4
        
        self.imName = imName
        self.img_label = qLabel
        self.img_label.setPixmap(QPixmap())

        self.setImage()
        self.img_label.mousePressEvent = self.getPos
        self.clicks = []

        self.zoomed = False
        self.scale_x = self.img_label.width()/20
        self.scale_y = self.img_label.height()/20


    def setImage(self): 
        self.img = QImage(self.imName)
        pixmap = QPixmap(QPixmap.fromImage(self.img))
        self.img_label.setPixmap(pixmap)


    def getPos(self , event):
        """Called on a mouse press event for the image; adds the clicked point to a points list.

        Args:
            event: mousePressEvent
        """
        if self.zoomed:
            new_x = event.pos().x()
            new_y = event.pos().y()

            self.zoomed = False
            self.setImage()
            # figure out x, y conversion
            x = new_x / 10 + (self.x - self.scale_x)
            y = new_y / 10 + (self.y - self.scale_y)
            
            rospy.loginfo(f"Got click: {x}, {y}")
            self.clicks.append([x,y])
            self.clickCount = self.clickCount - 1

            if self.clickCount == 0:
                self.shutdown()
            else:
                self.drawPoint(x,y)

        else:
            self.x = event.pos().x()
            self.y = event.pos().y()
            
            self.zoomed = True
            self.zoom(self.x,self.y)

            

    def drawPoint(self, x, y):
        im = cv2.imread(self.imName)
        im = cv2.circle(im, (round(x),round(y)), 2, (255,0,0), 3) 
        cv2.imwrite(self.imName, im)
        self.setImage()

    def zoom(self, x, y):
        # get zoomed in version of image:
        self.img_label.setPixmap(QPixmap())


        self.zoomed = True
        xrange = [round(x-self.scale_x), round(x+self.scale_x)]
        yrange = [round(y-self.scale_y), round(y+self.scale_y)]
        new_im = cv2.imread(self.imName)
        new_im = cv2.cvtColor(new_im, cv2.COLOR_BGR2RGB)
        new_im = new_im[yrange[0]:yrange[1],xrange[0]:xrange[1]]
        h, w, ch = new_im.shape
        b = ch * w

        QIm = QImage(new_im.data.tobytes(), w, h, b, QImage.Format_RGB888)
        pixmap = QPixmap(QPixmap.fromImage(QIm))
        self.img_label.setPixmap(pixmap.scaled(self.img_label.width(), self.img_label.height()))

        return x,y
        
    def shutdown(self):
        """Shuts down the click window once we have enough clicks, and sends info to the tracking node."""
        for pt in self.clicks:
            pt2d = Point2D()
            pt2d.x = pt[0]
            pt2d.y = pt[1]
            self.Distance.plane_points.append(pt2d)

        self.img_label.mousePressEvent = None
        
