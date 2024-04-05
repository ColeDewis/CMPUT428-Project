import os
import rospy
import rospkg

#from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
#from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel
from PyQt5.QtGui import QPixmap, QImage
from PyQt5 import QtCore
from sensor_msgs.msg import Image
# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Empty, Int32
from custom_msgs.msg import TrackRequest, ErrorDefinition, Point2D, DistanceDefinition
import cv2   
from rqt_custom_gui.trackerWidget import TrackerPlace
from rqt_custom_gui.distanceWidget import DistancePlace



class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        self.TrackerType = None # 0 1 2 3 fp fl tp tl
        self.load_ui()

        self.camIndices = [0,1] 
        self.camIndices = []
        cam_idx_sub = rospy.Subscriber("/cam_idx", Int32, self.idx_cb)
        while len(self.camIndices) < 2:
            continue
        cam_idx_sub.unregister()
        
        rospy.loginfo(f"Cam Indices: {self.camIndices}")


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

        self.ui.PlaceTrackers.clicked.connect(self.PlaceDistanceClick)
        self.ui.DoneDis.clicked.connect(self.DistanceDone)
        self.ui.ResetDis.clicked.connect(self.DistanceReset)

        self.buttons = [self.ui.PtoPButton,self.ui.PtoLButton,self.ui.LtoLButton,self.ui.FPButton,self.ui.FLButton,
                        self.ui.TPButton,self.ui.TLButton,self.ui.ResetButton,self.ui.InitButton,self.ui.GoButton,
                        self.ui.PlaceTrackers, self.ui.DoneDis, self.ui.ResetDis]

        self.bridge = CvBridge()
        self.error_req1 = ErrorDefinition()
        self.error_req2 = ErrorDefinition()
        self.trackers_placed = 0
        self.pointsplaced1 = len(self.error_req1.components)
        self.pointsplaced2 = len(self.error_req2.components)

        self.Distance1 = DistanceDefinition()
        self.Distance2 = DistanceDefinition()
        self.distance1inprog, self.distance2inprog = False,False

        self.trackersDisable(True)
        self.initDisable(True)
        self.goDisable(True)

        self.imSub1 = rospy.Subscriber("/cameras/cam%s" % (self.camIndices[0]), Image, self.updateIm1)
        self.imSub2 = rospy.Subscriber("/cameras/cam%s" % (self.camIndices[1]), Image, self.updateIm2)

        self.sizeSet = False
        self.sizeSet1 = False
        self.sizeSet2 = False
        self.notpaused = True

        self.error_req_pub = rospy.Publisher("/tracking_node/error_request", ErrorDefinition, queue_size=10)
        self.vs_start_pub = rospy.Publisher("/vs_start", Empty, queue_size=10)

    def idx_cb(self, data):
        if data.data not in self.camIndices: 
            self.camIndices.append(data.data)

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

    def PlaceDistanceClick(self):

        self.notpaused = False
        self.distance1inprog = True
        self.distance2inprog = True

        msg1 = rospy.wait_for_message("/cameras/cam%s" % (self.camIndices[0]), Image)
        msg2 = rospy.wait_for_message("/cameras/cam%s" % (self.camIndices[1]), Image)

        # msg1 = rospy.wait_for_message("img_pub_node", Image) # subscribe to the whatsapp topic and get the message
        # msg2 = rospy.wait_for_message("img_pub_node", Image) 

        cv2_img = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
        cv2.imwrite('catkin_ws/src/rqt_custom_gui/resource/im1.jpg', cv2_img)
        DistancePlace('catkin_ws/src/rqt_custom_gui/resource/im1.jpg', self.Distance1,self.ui.im_1)
        
        cv2_img = self.bridge.imgmsg_to_cv2(msg2, "bgr8")
        cv2.imwrite('catkin_ws/src/rqt_custom_gui/resource/im2.jpg', cv2_img)
        DistancePlace('catkin_ws/src/rqt_custom_gui/resource/im2.jpg', self.Distance2, self.ui.im_2 )
        

    def DistanceDone(self):
        '''int8 desired_distance
        int8 direction  # 0, 1, 2, 3 ? to define direction relative to the point
        float64 reference_distance_u  # real world dist between the rectangle side defined by p1-p2
        float64 reference_distance_v  # real world dist between the rectangle side defined by p2-p3'''
        if self.ui.Dim1Val.text():
            u = float(self.ui.Dim1Val.text())
        else:
            u = 0.0
        if self.ui.Dim2Val.text():
            v = float(self.ui.Dim2Val.text())
        else:
            v = 0.0
        dis = int(self.ui.TaskDistance.text())
        direct1 = int(self.ui.TaskDirection1.text())
        direct2 = int(self.ui.TaskDirection2.text())

        self.Distance1.reference_distance_u = u
        self.Distance1.reference_distance_v = v
        self.Distance2.reference_distance_u = u
        self.Distance2.reference_distance_v = v

        '''self.Distance1.desired_distance = dis
        self.Distance2.desired_distance = dis
        self.Distance1.direction = direct1
        self.Distance2.direction = direct2

        self.error_req1.distance_info = self.Distance1
        self.error_req2.distance_info = self.Distance2'''

        self.ui.PlaceTrackers.setDisabled(True)
        self.ui.DoneDis.setDisabled(True)
        self.ui.DoneDis.setStyleSheet("background-color : green")
        self.notpaused = True
            
    
    def DistanceReset(self):
        self.ui.PlaceTrackers.setDisabled(False)
        self.ui.DoneDis.setDisabled(False)
        self.ui.DoneDis.setStyleSheet("background-color : none")
        self.notpaused = True
        self.Distance1 = DistanceDefinition()
        self.Distance2 = DistanceDefinition()


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
        """Init tracker Button click listener. """
        rospy.loginfo("Sending error info stuff")
        
        # get distance and direction for the task
        dis = int(self.ui.TaskDistance.text())
        direct1 = int(self.ui.TaskDirection1.text())
        direct2 = int(self.ui.TaskDirection2.text())
        self.Distance1.desired_distance = dis
        self.Distance1.direction = direct1
        self.Distance2.desired_distance = dis
        self.Distance2.direction = direct2
        
        # publish error requests
        self.error_req1.distance_info = self.Distance1
        self.error_req2.distance_info = self.Distance2
        self.error_req_pub.publish(self.error_req1)
        self.error_req_pub.publish(self.error_req2)
        
        # update GUI
        self.goDisable(False)
        self.ui.InitButton.setStyleSheet("background-color : green")
        self.initDisable(True)
        self.ResetButtonClick()
        self.goDisable(False)
        self.imSub1.unregister()
        self.imSub1 = rospy.Subscriber("/cameras/cam%s/tracked_points" % (self.camIndices[0]), Image, self.updateIm1)
        self.imSub2.unregister()
        self.imSub2 = rospy.Subscriber("/cameras/cam%s/tracked_points" % (self.camIndices[1]), Image, self.updateIm2)

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

    def GoButtonClick(self):
        """Visual servo "go" button, sends vs_start command."""
        rospy.loginfo("Sending Visual Servo Start!")
        self.vs_start_pub.publish(Empty())

    def initTrackers(self):
        """Init Trackers button listener; opens a tracker place window to place the trackers."""
        if self.TrackerType is not None:
            self.pointsplaced1 = len(self.error_req1.components)
            self.pointsplaced2 = len(self.error_req2.components)

            self.trackers_placed += 1
            self.notpaused = False
            
            msg1 = rospy.wait_for_message("/cameras/cam%s" % (self.camIndices[0]), Image)
            msg2 = rospy.wait_for_message("/cameras/cam%s" % (self.camIndices[1]), Image)

            # msg1 = rospy.wait_for_message("img_pub_node", Image) # subscribe to the whatsapp topic and get the message
            # msg2 = rospy.wait_for_message("img_pub_node", Image) 

            cv2_img = self.bridge.imgmsg_to_cv2(msg1, "bgr8")
            cv2.imwrite('catkin_ws/src/rqt_custom_gui/resource/im1.jpg', cv2_img)
            TrackerPlace(self.TrackerType, 'catkin_ws/src/rqt_custom_gui/resource/im1.jpg', self.error_req1,self.ui.im_1)
            
            cv2_img = self.bridge.imgmsg_to_cv2(msg2, "bgr8")
            cv2.imwrite('catkin_ws/src/rqt_custom_gui/resource/im2.jpg', cv2_img)
            TrackerPlace(self.TrackerType, 'catkin_ws/src/rqt_custom_gui/resource/im2.jpg', self.error_req2,self.ui.im_2)
            
            
            if self.trackers_placed == 2:
                self.initDisable(False)


    def updateIm1(self,data: Image):
        if self.notpaused:
            frame = self.bridge.imgmsg_to_cv2(img_msg=data, desired_encoding="rgb8")
            #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            b = ch * w
            QIm = QImage(frame.data, w, h, b, QImage.Format_RGB888)
            if not self.sizeSet:
                self.sizeSet1 = True
                if self.sizeSet2:
                    self.setImLayouts(w,h,1)
                else:
                    self.h = h
                    self.w = w+10

            
            pixmap = QPixmap(QPixmap.fromImage(QIm))
            self.ui.im_1.setPixmap(pixmap)
            

        elif self.pointsplaced1-len(self.error_req1.components)==-1 and self.pointsplaced2-len(self.error_req2.components)==-1:
            self.pointsplaced2 = len(self.error_req2.components)
            self.pointsplaced1 = len(self.error_req1.components)
            self.TrackerType = None
            self.notpaused = True
        elif self.distance1inprog:
            if  len(self.Distance1.plane_points) ==  4:
                self.distance1inprog = False
                if not self.distance2inprog:
                    self.notpaused = True

        

    def updateIm2(self,data: Image):
        if self.notpaused:
            frame = self.bridge.imgmsg_to_cv2(img_msg=data, desired_encoding="rgb8")
            #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            h, w, ch = frame.shape
            b = ch * w
            QIm = QImage(frame.data, w, h, b, QImage.Format_RGB888)
            pixmap = QPixmap(QPixmap.fromImage(QIm))
            self.ui.im_2.setPixmap(pixmap)
            if not self.sizeSet:
                self.sizeSet2 = True
                if self.sizeSet1:
                    self.setImLayouts(w,h,2)
                else:
                    self.h = h
                    self.w = w+10
        elif self.pointsplaced2-len(self.error_req2.components)==-1 and self.pointsplaced1-len(self.error_req1.components)==-1: 
            self.pointsplaced2 = len(self.error_req2.components)
            self.pointsplaced1 = len(self.error_req1.components)
            self.TrackerType = None
            self.notpaused = True
        elif self.distance2inprog:
            if  len(self.Distance2.plane_points) ==  4:
                self.distance2inprog = False
                if not self.distance1inprog:
                    self.notpaused = True

    def setImLayouts(self,w,h, f):
        if f == 1:
            self.ui.im_1.setGeometry(QtCore.QRect(self.ui.im_1.x(), self.ui.im_1.y(),w, h)) 
            self.ui.im_2.setGeometry(QtCore.QRect(self.ui.im_2.x(), self.ui.im_2.y(),self.w, self.h)) 
        elif f == 2:
            self.ui.im_2.setGeometry(QtCore.QRect(self.ui.im_2.x(), self.ui.im_2.y(),w, h)) 
            self.ui.im_1.setGeometry(QtCore.QRect(self.ui.im_1.x(), self.ui.im_1.y(),self.w, self.h)) 
        self.ui.im_layouts.setGeometry(QtCore.QRect(self.ui.im_layouts.x(),self.ui.im_layouts.y(), w + self.w + 10, max(h, self.h)+10))
         
        self.sizeSet = True