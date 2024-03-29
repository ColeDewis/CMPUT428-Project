import sys
import rospy
import cv2
import cv_bridge
import rospkg
import numpy as np

from sensor_msgs.msg import Image

class ImgPubNode:
    """Basic Node publishes an image repeatedly to to ros."""
    def __init__(self):
        """Initialize a camera node by getting the rosparam and opening the opencv stream."""
        
        rospy.init_node("img_pub_node", anonymous=True)
        
        # setup opencv input
        pkg = rospkg.RosPack()
        path = pkg.get_path("camera_node")
        rospy.loginfo(path)
        
        self.base_img = cv2.imread(f"{path}/img/ruler_img.jpg") # CHANGE IMAGE NAME HERE!
        self.base_img2 = cv2.imread(f"{path}/img/cool_patch.jpg") # CHANGE IMAGE NAME HERE!

        self.base_img = cv2.cvtColor(self.base_img, cv2.COLOR_BGR2RGB)
        self.base_img2 = cv2.cvtColor(self.base_img2, cv2.COLOR_BGR2RGB)

        self.bridge = cv_bridge.CvBridge()
        self.image = self.bridge.cv2_to_imgmsg(self.base_img, encoding="rgb8")
        self.image2 = self.bridge.cv2_to_imgmsg(self.base_img2, encoding="rgb8")
        # --- Publishers ---
        self.image_pub = rospy.Publisher("img_pub_node", Image, queue_size=10)
        
        # timer for image update
        rospy.Timer(rospy.Duration(0.1), self.update_callback)
        self.check = 0
        
        
    def update_callback(self, event=None):
        """Callback to update the image on the camera topic."""
        if self.check == 0:
            self.image_pub.publish(self.image)
            self.check = 1
        else:
            self.image_pub.publish(self.image2)
            self.check = 0
        

def main(args):
    rospy.loginfo("Starting a ImgPubNode ...")
    node = ImgPubNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down ImgPubNode...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)