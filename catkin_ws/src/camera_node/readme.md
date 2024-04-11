# Camera Node
Package that contains camera related functionality. Contains the following
`camera_node.py`: Very basic camera node for ROS. Run it with `rosrun camera_node camera_node.py _cam_idx:=<camera index>`

`img_pub_node.py`: Node that publishes a static image, for testing purposes when cameras are not available.

`tracking_node.py`: Node that manages trackers on camera feeds. Can listen and track on multiple tracking feeds. Also manages image error for visual servoing. Run with `rosrun camera_node tracking_node.py _idxs:="[idx1, idx2, ...]"`
