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
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty
from custom_msgs.msg import TrackRequest, ErrorDefinition, Point2D
import cv2

class TrackerPlace(QWidget):
    def __init__(self, trackerType, imName, error_req):
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
        
        self.imName = imName
        self.load_ui()
        self.img_label.mousePressEvent = self.getPos
        self.clicks = []


    def setImage(self): 
        self.img = QImage(self.imName)
        pixmap = QPixmap(QPixmap.fromImage(self.img))
        self.img_label.setPixmap(pixmap)
        self.resize(self.img.size())
        self.show()

    def load_ui(self):
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_custom_gui'), 'resource', 'trackerPlace.ui')
        self.ui = loadUi(ui_file, self)
        self.img_label = self.ui.mainLabel
        self.setImage()

    def getPos(self , event):
        """Called on a mouse press event for the image; adds the clicked point to a points list.

        Args:
            event: mousePressEvent
        """
        x = event.pos().x()
        y = event.pos().y()
        rospy.loginfo(f"Got click: {x}, {y}")
        self.clicks.append([x,y])
        self.clickCount = self.clickCount - 1
        if self.clickCount == 0:
            self.shutdown()
        else:
            self.drawPoint(x,y)

    def drawPoint(self, x, y):
        im = cv2.imread(self.imName)
        im = cv2.circle(im, (x,y), 3, (255,0,0), 5) 
        cv2.imwrite(self.imName, im)
        self.setImage()

        

    
    def shutdown(self):
        """Shuts down the click window once we have enough clicks, and sends info to the tracking node."""
        for pt in self.clicks:
            pt2d = Point2D()
            pt2d.x = pt[0]
            pt2d.y = pt[1]
            self.track_req.points.append(pt2d)
        self.error_req.components.append(self.track_req)
        self.close()