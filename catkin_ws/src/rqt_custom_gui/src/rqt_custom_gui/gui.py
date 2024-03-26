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

            self.bridge = CvBridge()
    
        def load_ui(self):
            ui_file = os.path.join(rospkg.RosPack().get_path('rqt_custom_gui'), 'resource', 'form.ui')
            self.ui = loadUi(ui_file, self)

    
            
        def PtoPClick(self):
            pass
        
        def PtoLClick(self):
            pass

        def LtoLClick(self):
            pass
            
        def FixedPointClick(self):
            self.TrackerType = 0

        def FixedLineClick(self):
            self.TrackerType = 1
        
        def TrackPointClick(self):
            self.TrackerType = 2
        
        def TrackLineClick(self):
            self.TrackerType = 3

        def GoButtonClick(self):
            pass

        def ResetButtonClick(self):
            pass

        def initTrackers(self):
            if self.TrackerType is not None:
                msg = rospy.wait_for_message("img_pub_node", Image) # subscribe to the whatsapp topic and get the message
                cv2_img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                cv2.imwrite('catkin_ws/src/rqt_custom_gui/resource/im1.jpg', cv2_img)
                print("Nice")
                tracker_widget_1 = TrackerPlace(self.TrackerType, 'catkin_ws/src/rqt_custom_gui/resource/im1.jpg')
                
                self.TrackerType = None


class TrackerPlace(QWidget):
        def __init__(self, trackerType,imName):
            super(TrackerPlace, self).__init__()

            
            self.clickCount = 1 if trackerType in {0,2} else 2
            self.load_ui()
            
            self.img = QImage(imName)
            pixmap = QPixmap(QPixmap.fromImage(self.img))
            img_label = self.ui.mainLabel
            img_label.setPixmap(pixmap)
            img_label.mousePressEvent = self.getPos
            self.clicks = []

            self.show()
    
        def load_ui(self):
            ui_file = os.path.join(rospkg.RosPack().get_path('rqt_custom_gui'), 'resource', 'trackerPlace.ui')
            self.ui = loadUi(ui_file, self)

        def getPos(self , event):
            x = event.pos().x()
            y = event.pos().y()
            print(x,y)
            self.clicks.append([x,y])
            


    