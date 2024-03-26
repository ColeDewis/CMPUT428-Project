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
from rqt_custom_gui.trackerWidget import TrackerPlace



class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        self.TrackerType = None # 0 1 2 3 fp fl tp tl
        self.load_ui()

        self.camIndices = [0,1]

        self.ui.PtoPButton.clicked.connect(self.PtoPClick)
        self.ui.PtoLButton.clicked.connect(self.PtoLClick)
        self.ui.LtoLButton.clicked.connect(self.LtoLClick)

        self.ui.FPButton.clicked.connect(self.FixedPointClick)
        self.ui.FLButton.clicked.connect(self.FixedLineClick)
        self.ui.TPButton.clicked.connect(self.TrackPointClick)
        self.ui.TLButton.clicked.connect(self.TrackLineClick)
        self.ui.ResetButton.clicked.connect(self.ResetButtonClick)
        self.ui.InitButton.clicked.connect(self.InitButtonClick)
        self.ui.GoButton.clicked.connect(self.GoButtonClick)

        self.buttons = [self.ui.PtoPButton,self.ui.PtoLButton,self.ui.LtoLButton,self.ui.FPButton,self.ui.FLButton,
                        self.ui.TPButton,self.ui.TLButton,self.ui.ResetButton,self.ui.InitButton,self.ui.GoButton]

        self.bridge = CvBridge()
        self.error_req1 = ErrorDefinition()
        self.error_req2 = ErrorDefinition()
        self.trackers_placed = 0

        self.trackersDisable(True)
        self.initDisable(True)
        self.goDisable(True)

    def tasksDisable(self, i):
        self.ui.PtoPButton.setDisabled(i)
        self.ui.PtoLButton.setDisabled(i)
        self.ui.LtoLButton.setDisabled(i)

    def trackersDisable(self, i):
        self.ui.FPButton.setDisabled(i)
        self.ui.FLButton.setDisabled(i)
        self.ui.TPButton.setDisabled(i)
        self.ui.TLButton.setDisabled(i)

    def initDisable(self,i):
        self.ui.InitButton.setDisabled(i)

    def goDisable(self, i):
        self.ui.GoButton.setDisabled(i)

    def load_ui(self):
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_custom_gui'), 'resource', 'form.ui')
        self.ui = loadUi(ui_file, self)

    def PtoPClick(self):
        self.error_req1.type = "ptpt"
        self.error_req2.type = "ptpt"
        self.error_req1.cam_idx = self.camIndices[0]
        self.error_req2.cam_idx = self.camIndices[1]
        self.error_req1.components = []
        self.error_req2.components = []
        self.ui.FPButton.setDisabled(False)
        self.ui.TPButton.setDisabled(False)
        self.ui.PtoPButton.setStyleSheet("background-color : green")
        self.tasksDisable(True)

    
    def PtoLClick(self):
        self.error_req1.type = "ptln"
        self.error_req2.type = "ptln"
        self.error_req1.cam_idx = self.camIndices[0]
        self.error_req2.cam_idx = self.camIndices[1]
        self.error_req1.components = []
        self.error_req2.components = []
        self.tasksDisable(True)
        self.ui.PtoLButton.setStyleSheet("background-color : green")
        self.trackersDisable(False)

    def LtoLClick(self):
        self.error_req1.type = "lnln"
        self.error_req2.type = "lnln"
        self.error_req1.cam_idx = self.camIndices[0]
        self.error_req2.cam_idx = self.camIndices[1]
        self.error_req1.components = []
        self.error_req2.components = []
        self.ui.FLButton.setDisabled(False)
        self.ui.TLButton.setDisabled(False)
        self.ui.LtoLButton.setStyleSheet("background-color : green")
        self.tasksDisable(True)
        
    def FixedPointClick(self):
        """Fixed point button click listener; sets selected tracker type to fixed point."""
        self.TrackerType = 0
        self.ui.FPButton.setStyleSheet("background-color : green")
        rospy.loginfo("Tracker Type Selected: Fixed point")
        self.ui.FLButton.setDisabled(True)
        self.ui.FPButton.setDisabled(True)
        if self.error_req1.type == "ptln":
            self.ui.TPButton.setDisabled(True)
        self.initTrackers()

    def FixedLineClick(self):
        """Fixed line button click listener; sets selected tracker type to fixed line."""
        self.TrackerType = 1
        self.ui.FLButton.setStyleSheet("background-color : green")
        rospy.loginfo("Tracker Type Selected: Fixed line")
        self.ui.FLButton.setDisabled(True)
        self.ui.FPButton.setDisabled(True)
        if self.error_req1.type == "ptln":
            self.ui.TLButton.setDisabled(True)
        self.initTrackers()
    
    def TrackPointClick(self):
        """Track point button click listener; sets selected tracker type to track-point."""
        self.TrackerType = 2
        self.ui.TPButton.setStyleSheet("background-color : green")
        rospy.loginfo("Tracker Type Selected: Tracked point")
        self.ui.TLButton.setDisabled(True)
        self.ui.TPButton.setDisabled(True)
        if self.error_req1.type == "ptln":
            self.ui.FPButton.setDisabled(True)
        self.initTrackers()
    
    def TrackLineClick(self):
        """Track Line button click listener; sets selected tracker type to track-line."""
        self.TrackerType = 3
        self.ui.TLButton.setStyleSheet("background-color : green")
        rospy.loginfo("Tracker Type Selected: Tracked line")
        self.ui.TLButton.setDisabled(True)
        self.ui.TPButton.setDisabled(True)
        if self.error_req1.type == "ptln":
            self.ui.FLButton.setDisabled(True)
        self.initTrackers()

    def InitButtonClick(self):
        """GO Button click listener. """
        # rospy.loginfo("Sending Visual Servo Start!")
        # rospy.Publisher("/vs_start", Empty).publish(Empty())
        rospy.loginfo("Sending error info stuff")
        rospy.Publisher("/tracking_node/error_request", ErrorDefinition, queue_size=10).publish(self.error_req1)
        rospy.Publisher("/tracking_node/error_request", ErrorDefinition, queue_size=10).publish(self.error_req2)
        self.goDisable(False)
        self.ui.InitButton.setStyleSheet("background-color : green")
        self.initDisable(True)
        self.ResetButtonClick()
        self.goDisable(False)

    def ResetButtonClick(self):
        """Reset any progress."""
        #TODO Fill the rest of this out 
        self.tasksDisable(False)
        self.trackersDisable(True)
        self.initDisable(True)
        self.goDisable(True)
        rospy.loginfo(self.error_req1.components)
        rospy.loginfo(self.error_req2.components)
        self.error_req1 = ErrorDefinition()
        self.error_req2 = ErrorDefinition()
        self.trackers_placed = 0
        self.TrackerType = None
        for i in self.buttons:
            i.setStyleSheet("background-color : none")

    def GoButtonClick(Self):
        """I'm not 100% sure how you want to start the visual servoing but do it here"""
        # TODO Cole help me Cole
        pass

    def initTrackers(self):
        """Init Trackers button listener; opens a tracker place window to place the trackers."""
        if self.TrackerType is not None:
            self.trackers_placed += 1

            #msg1 = rospy.wait_for_message("/cameras/cam%s" % (self.camIndices[0]), Image)
            #msg2= rospy.wait_for_message("/cameras/cam%s" % (self.camIndices[1]), Image)

            msg1 = rospy.wait_for_message("img_pub_node", Image) # subscribe to the whatsapp topic and get the message
            msg2 = rospy.wait_for_message("img_pub_node", Image) 

            cv2_img = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
            cv2.imwrite('catkin_ws/src/rqt_custom_gui/resource/im1.jpg', cv2_img)
            tracker_place_widget = TrackerPlace(self.TrackerType, 'catkin_ws/src/rqt_custom_gui/resource/im1.jpg', self.error_req1)
            
            cv2_img = self.bridge.imgmsg_to_cv2(msg2, "bgr8")
            cv2.imwrite('catkin_ws/src/rqt_custom_gui/resource/im2.jpg', cv2_img)
            tracker_place_widget = TrackerPlace(self.TrackerType, 'catkin_ws/src/rqt_custom_gui/resource/im1.jpg', self.error_req2)
            
            self.TrackerType = None
            
            if self.trackers_placed == 2:
                self.initDisable(False)