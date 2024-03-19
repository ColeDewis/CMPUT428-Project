#!/usr/bin/python3

import numpy as np
import rospy

from kortex_bringup import KinovaGen3

if __name__ == "__main__":
    # Init ros node
    rospy.init_node('kortex_test_movements', anonymous=False)

    # Robot node
    gen3 = KinovaGen3()
      
    print()
    # 0: base rot
    # 1: arm angle 1
    # 3: arm angle 2
    # 5: arm angle 3
    angles = np.deg2rad(np.array([-30.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922]))
    print("BEFORE: ", rospy.get_rostime())
    success = gen3.send_joint_angles(angles)
    print("AFTER: ", rospy.get_rostime())
    print("Done. Joints:", gen3.position)