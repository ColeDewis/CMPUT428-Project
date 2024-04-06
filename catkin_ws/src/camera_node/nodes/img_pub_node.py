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
        self.base_imgs = []
        self.bridge = cv_bridge.CvBridge()

        
        '''for i in range(10, 24):
            base_img = cv2.imread(f"{path}/img/ruler_img.jpg") # CHANGE IMAGE NAME HERE!
            base_img = cv2.cvtColor(base_img, cv2.COLOR_BGR2RGB)
            image = self.bridge.cv2_to_imgmsg(base_img, encoding="rgb8")
            self.base_imgs.append(image)
'''
        for i in range(0, 10):
            base_img = cv2.imread(f"{path}/img/banana/banana-000"+str(i)+".jpg") # CHANGE IMAGE NAME HERE!
            base_img = cv2.cvtColor(base_img, cv2.COLOR_BGR2RGB)
            image = self.bridge.cv2_to_imgmsg(base_img, encoding="rgb8")
            self.base_imgs.append(image)

        for i in range(10, 38):
            base_img = cv2.imread(f"{path}/img/banana/banana-00"+str(i)+".jpg") # CHANGE IMAGE NAME HERE!
            base_img = cv2.cvtColor(base_img, cv2.COLOR_BGR2RGB)
            image = self.bridge.cv2_to_imgmsg(base_img, encoding="rgb8")
            self.base_imgs.append(image)
        

        '''for i in range(0, 10):
            base_img = cv2.imread(f"{path}/img/gnome/gnome-000"+str(i)+".jpg") # CHANGE IMAGE NAME HERE!
            base_img = cv2.cvtColor(base_img, cv2.COLOR_BGR2RGB)
            image = self.bridge.cv2_to_imgmsg(base_img, encoding="rgb8")
            self.base_imgs.append(image)

        for i in range(10, 24):
            base_img = cv2.imread(f"{path}/img/gnome/gnome-00"+str(i)+".jpg") # CHANGE IMAGE NAME HERE!
            base_img = cv2.cvtColor(base_img, cv2.COLOR_BGR2RGB)
            image = self.bridge.cv2_to_imgmsg(base_img, encoding="rgb8")
            self.base_imgs.append(image)'''
        
        # --- Publishers ---
        self.image_pub = rospy.Publisher("img_pub_node", Image, queue_size=10)
        
        # timer for image update
        rospy.Timer(rospy.Duration(0.1), self.update_callback)
        self.check = 0
        
        
    def update_callback(self, event=None):
        """Callback to update the image on the camera topic."""
        self.image_pub.publish(self.base_imgs[self.check])
        self.check += 1
        if self.check == len(self.base_imgs):
            self.check = 1

        

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