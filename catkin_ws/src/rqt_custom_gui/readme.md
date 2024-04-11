# rqt_custom_gui
rqt plugin that handles task specification. Contains the following
`gui.py`: Handles the driver code and setup

`buttonWidget.py`: Main widget, handles the majority of functions including all input fields (buttons and text), and calls the other two

`trackerWidget.py`: widget that handles grabbing image coordinates for the trackers

`distanceWidget.py`: widget that handles grabbing image coordinates for the distance specification

This plugin can be used by either runnin `rosrun rqt_custom_gui rqt_custom_gui` or by using rqt and selecting it manually from the dropdown menu.