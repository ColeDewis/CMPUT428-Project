import sys
import rospy
import cv2
import cv_bridge

from sensor_msgs.msg import Image


class CameraNode:
    """Basic Node that runs a camera on a given camera index and publishes to ros."""
    def __init__(self):
        """Initialize a camera node by getting the rosparam and opening the opencv stream."""
        
        rospy.init_node("camera_node")
        cam_param = rospy.search_param("cam_idx")
        self.cam_idx = rospy.get_param(cam_param, None)
        
        if self.cam_idx is None:
            rospy.logwarn("Must pass camera index as _cam_idx:=<cam_idx>")
            exit()
    
        rospy.loginfo(f"Initialized Camera on topic /cameras/cam{self.cam_idx}")
        
        # setup opencv input
        self.video_cap = cv2.VideoCapture(0)
        self.bridge = cv_bridge.CvBridge()
        
        # --- Publishers ---
        self.image_pub = rospy.Publisher(f"/cameras/cam{self.cam_idx}", Image, queue_size=10)
        
        # timer for image update
        rospy.Timer(rospy.Duration(0.1), self.update_callback)
        
    def update_callback(self, event=None):
        """Callback to update the image on the camera topic."""
        
        ret, frame = self.video_cap.read()
        if not ret:
            rospy.logwarn(f"Couldn't get image frame for camera {self.cam_idx}")
            return
            
        # convert image color format and publish
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image: Image = self.bridge.cv2_to_imgmsg(cvim=frame, encoding="rgb8")
        self.image_pub.publish(image)

def main(args):
    rospy.loginfo("Starting a CameraNode ...")
    node = CameraNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Camera...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)