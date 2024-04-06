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
from custom_msgs.msg import TrackRequest, ErrorDefinition, Point2D
import cv2

class TrackerPlace(QWidget):
    def __init__(self, trackerType, im, error_req,qLabel):
        super(TrackerPlace, self).__init__()
        self.error_req = error_req
        self.track_req = TrackRequest()
        self.track_req.points = []
        self.clickCount = 1 if trackerType in (0, 2) else 2
        if trackerType == 0:
            self.track_req.type = "fp"
        elif trackerType == 1:
            self.track_req.type = "fl"
        elif trackerType == 2:
            self.track_req.type = "tp"
        elif trackerType == 3:
            self.track_req.type = "tl"
        else:
            raise TypeError("invalid tracker type")
        
        self.img_label = qLabel
        self.im = im
        self.setImage()
        self.img_label.mouseReleaseEvent = self.getPos
        self.clicks = []

        self.zoomed = False
        self.scale_x = self.img_label.width()/20
        self.scale_y = self.img_label.height()/20


    def setImage(self): 
        h, w, ch = self.im.shape
        b = ch*w
        QIm = QImage(self.im.data.tobytes(), w, h, b,QImage.Format_RGB888)
        pixmap = QPixmap(QPixmap.fromImage(QIm))
        self.img_label.setPixmap(pixmap)


    def getPos(self , event):
        """Called on a mouse press event for the image; adds the clicked point to a points list.

        Args:
            event: mousePressEvent
        """

        if self.track_req.type == "tp" or self.track_req.type == "tl":
            x = event.pos().x()
            y = event.pos().y()
            rospy.loginfo(f"Got click: {x}, {y}")
            self.clicks.append([x,y])
            self.clickCount = self.clickCount - 1
            self.drawPoint(x,y)
            if self.clickCount == 0:
                self.shutdown()
                print("shutting down")
        else:
            if self.zoomed:
                new_x = event.pos().x()
                new_y = event.pos().y()

                self.zoomed = False
                self.setImage()
                # figure out x, y conversion
                x = new_x / 10 + self.xrange[0]
                y = new_y / 10 + self.yrange[0]
                
                rospy.loginfo(f"Got click: {x}, {y}")
                self.clicks.append([x,y])
                self.clickCount = self.clickCount - 1

                self.drawPoint(x,y)

                if self.clickCount == 0:
                    self.shutdown()
                

            else:
                self.x = event.pos().x()
                self.y = event.pos().y()
                
                self.zoomed = True
                self.zoom(self.x,self.y)


    def drawPoint(self, x, y):
        self.im = cv2.circle(self.im, (round(x),round(y)), 3, (255,0,0), 5) 
        self.setImage()


    def zoom(self, x, y):
        # get zoomed in version of image:
        self.img_label.setPixmap(QPixmap())

        if self.scale_x > x:
            self.xrange = [0, round(2*self.scale_x)]
        elif self.scale_x > len(self.im):
            self.xrange = [round(len(self.im) - 2*self.scale_x), self.im_size[1]]
        else:
            self.xrange = [round(x-self.scale_x), round(x+self.scale_x)]
        if self.scale_y > y:
            self.yrange = [0, round(2*self.scale_y)]
        elif self.scale_y > len(self.im[0]):
            self.yrange = [round(len(self.im[0]) - 2*self.scale_y), self.img.height()]
        else:
            self.yrange = [round(y-self.scale_y), round(y+self.scale_y)]
            
        self.zoomed = True

        #new_im = cv2.imread(self.imName)
        #new_im = cv2.cvtColor(new_im, cv2.COLOR_BGR2RGB)
        new_im = self.im.copy()
        new_im = new_im[self.yrange[0]:self.yrange[1],self.xrange[0]:self.xrange[1]]
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
            pt2d.x = p[0]
            pt2d.y = p[1]
            self.track_req.points.append(pt2d)
        self.error_req.components.append(self.track_req)
        self.img_label.mousePressEvent = None
