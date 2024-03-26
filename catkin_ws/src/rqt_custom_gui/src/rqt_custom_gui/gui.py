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

class CustomGUI(Plugin):

    def __init__(self, context):
        super(CustomGUI, self).__init__(context)
        # Give QObjects reasonable names
        self.setObjectName('CustomGui')
        self.layout = QVBoxLayout()
        

        # Process standalone plugin command-line arguments
        from argparse import ArgumentParser
        parser = ArgumentParser()
        # Add argument(s) to the parser.
        parser.add_argument("-q", "--quiet", action="store_true",
                      dest="quiet",
                      help="Put plugin in silent mode")
        args, unknowns = parser.parse_known_args(context.argv())
        if not args.quiet:
            print('arguments: ', args)
            
        # Create QWidget
        self._widget = MyWidget()

        context.add_widget(self._widget)



    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass

    #def trigger_configuration(self):
        # Comment in to signal that the plugin has a way to configure
        # This will enable a setting button (gear icon) in each dock widget title bar
        # Usually used to open a modal configuration dialog
    
class MyWidget(QWidget):
    def __init__(self):
        super(MyWidget, self).__init__()

        self.TrackerType = None # 0 1 2 3 fp fl tp tl
        self.load_ui()
        #self.subscriber = 

        self.ui.PtoPButton.clicked.connect(self.PtoPClick)
        self.ui.PtoLButton.clicked.connect(self.PtoLClick)
        self.ui.LtoLButton.clicked.connect(self.LtoLClick)

        self.ui.FPButton.clicked.connect(self.FixedPointClick)
        self.ui.FLButton.clicked.connect(self.FixedLineClick)
        self.ui.TPButton.clicked.connect(self.TrackPointClick)
        self.ui.TLButton.clicked.connect(self.TrackLineClick)
        self.ui.TrackerGoButton.clicked.connect(self.initTrackers)
        self.ui.ResetButton.clicked.connect(self.ResetButtonClick)
        self.ui.GoButton.clicked.connect(self.GoButtonClick)

        self.bridge = CvBridge()
        self.error_req = ErrorDefinition()

    def load_ui(self):
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_custom_gui'), 'resource', 'form.ui')
        self.ui = loadUi(ui_file, self)


    def PtoPClick(self):
        self.error_req.type = "ptpt"
        self.error_req.cam_idx = 2
        self.error_req.components = []
    
    def PtoLClick(self):
        self.error_req.type = "ptln"
        self.error_req.cam_idx = 2
        self.error_req.components = []

    def LtoLClick(self):
        self.error_req.type = "lnln"
        self.error_req.cam_idx = 2
        self.error_req.components = []
        
    def FixedPointClick(self):
        """Fixed point button click listener; sets selected tracker type to fixed point."""
        self.TrackerType = 0
        rospy.loginfo("Tracker Type Selected: Fixed point")

    def FixedLineClick(self):
        """Fixed line button click listener; sets selected tracker type to fixed line."""
        self.TrackerType = 1
        rospy.loginfo("Tracker Type Selected: Fixed line")
    
    def TrackPointClick(self):
        """Track point button click listener; sets selected tracker type to track-point."""
        self.TrackerType = 2
        rospy.loginfo("Tracker Type Selected: Tracked point")
    
    def TrackLineClick(self):
        """Track Line button click listener; sets selected tracker type to track-line."""
        self.TrackerType = 3
        rospy.loginfo("Tracker Type Selected: Tracked line")

    def GoButtonClick(self):
        """GO Button click listener. """
        # rospy.loginfo("Sending Visual Servo Start!")
        # rospy.Publisher("/vs_start", Empty).publish(Empty())
        rospy.loginfo("Sending error info stuff")
        rospy.Publisher("/tracking_node/error_request", ErrorDefinition, queue_size=10).publish(self.error_req)

    def ResetButtonClick(self):
        """Reset any progress."""
        rospy.loginfo(self.error_req.components)
        self.error_req = ErrorDefinition()

    def initTrackers(self):
        """Init Trackers button listener; opens a tracker place window to place the trackers."""
        if self.TrackerType is not None:
            msg = rospy.wait_for_message("/cameras/cam2", Image) # subscribe to the whatsapp topic and get the message
            cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite('catkin_ws/src/rqt_custom_gui/resource/im1.jpg', cv2_img)
            print("Nice")
            tracker_place_widget = TrackerPlace(self.TrackerType, 'catkin_ws/src/rqt_custom_gui/resource/im1.jpg', self.error_req)
            
            self.TrackerType = None

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
        
        self.load_ui()
        self.img = QImage(imName)
        pixmap = QPixmap(QPixmap.fromImage(self.img))
        img_label = self.ui.mainLabel
        img_label.setPixmap(pixmap)
        img_label.mousePressEvent = self.getPos
        self.clicks = []
        self.resize(self.img.size())
        self.show()

    def load_ui(self):
        ui_file = os.path.join(rospkg.RosPack().get_path('rqt_custom_gui'), 'resource', 'trackerPlace.ui')
        self.ui = loadUi(ui_file, self)

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
    
    def shutdown(self):
        """Shuts down the click window once we have enough clicks, and sends info to the tracking node."""
        for pt in self.clicks:
            pt2d = Point2D()
            pt2d.x = pt[0]
            pt2d.y = pt[1]
            self.track_req.points.append(pt2d)
        self.error_req.components.append(self.track_req)
        self.close()