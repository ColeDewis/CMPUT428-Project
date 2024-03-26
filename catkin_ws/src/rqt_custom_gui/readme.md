This plugin is a series of buttons that do very specific things. Best way to set up your dashboard is like this:

| -------------------------------------|
|         Custom_Gui                   |
|                                      |
|--------------------------------------|
|                  |                   |
|                  |                   |
|                  |                   |
|   Im_View_One    |  Im_View_Two      |
|                  |                   |
|                  |                   |
----------------------------------------

Select the type of tracker to place, then click Place Selected Trackers.
Place them in the same place on the 2 windows that open.

COLE READ HERE

All of the code is in src/rqt_custom_gui/gui.py. The class customgui is the plugin which calls widget with the buttons

MyWidget is the widget with the buttons. All the button onclick functions are at the bottom, most of them have `pass`
except for the tracker ones which specify tracker type (required to open window)

TrackerPlace is the window that opens up, all it does so far is print out the location of the last click. Will need
to change that so it can save the location of multiple clicks. 

