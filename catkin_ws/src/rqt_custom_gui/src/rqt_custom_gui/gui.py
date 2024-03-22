import os
import rospy
import rospkg

#from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
#from python_qt_binding.QtWidgets import QWidget
from rqt_gui_py.plugin import Plugin
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QPushButton, QLabel

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
            self.load_ui()

            self.ui.PtoPButton.clicked.connect(self.PtoPClick)
            self.ui.PtoLButton.clicked.connect(self.PtoLClick)
            self.ui.LtoLButton.clicked.connect(self.LtoLClick)

            self.ui.FPButton.clicked.connect(self.FixedPointClick)
            self.ui.FLButton.clicked.connect(self.FixedLineClick)
            self.ui.TPButton.clicked.connect(self.TrackPointClick)
            self.ui.TLButton.clicked.connect(self.TrackLineClick)
    
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
            print("Nice")

        def FixedLineClick(self):
            pass
        
        def TrackPointClick(self):
            pass
        
        def TrackLineClick(self):
            pass

        def GoButtonClick(self):
            pass

        def ResetButtonClick(self):
            pass