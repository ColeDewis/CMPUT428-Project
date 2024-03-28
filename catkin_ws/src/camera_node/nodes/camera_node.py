import sys
import rospy
import cv2
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image
from std_msgs.msg import Int32
from visual_servoing.msg import Error, TrackedPoint, TrackedPoints

import matplotlib.pyplot as plt

class CameraNode:
    """Basic Node that runs a camera on a given camera index and publishes to ros."""
    def __init__(self):
        """Initialize a camera node by getting the rosparam and opening the opencv stream."""
        
        rospy.init_node("camera_node", anonymous=True)
        cam_param = rospy.search_param("cam_idx")
        self.cam_idx = rospy.get_param(cam_param, None)
                
        if self.cam_idx is None:
            rospy.logwarn("Must pass camera index as _cam_idx:=<cam_idx>")
            exit()
        
        idx_pub = rospy.Publisher("cam_idx", Int32)
        while not rospy.is_shutdown():
            if idx_pub.get_num_connections() > 0:
                idx_pub.publish(self.cam_idx)
                idx_pub.unregister()
                break
        # node_name = f"camera_node{self.cam_idx}"
    
        rospy.delete_param(cam_param) # delete param so its needed for future runs.
        rospy.loginfo(f"Initialized Camera on topic /cameras/cam{self.cam_idx}")
        
        # setup opencv input
        self.video_cap = cv2.VideoCapture(self.cam_idx)
        self.bridge = cv_bridge.CvBridge()
        
        self.last_points = None
        self.last_frame = None
        
        self.trackers_running = False
        
        self.target = np.array([520, 170])
        
        # --- Publishers ---
        self.image_pub = rospy.Publisher(f"/cameras/cam{self.cam_idx}", Image, queue_size=10)
        self.tracked_image_pub = rospy.Publisher(f"/cameras/cam{self.cam_idx}/tracked_points", Image, queue_size=10)
        
        self.tracked_points_pub = rospy.Publisher("/eef_pos", TrackedPoints, queue_size=10)
        
        self.image_error_pub = rospy.Publisher("/image_error", Error, queue_size=10)
        
        # timer for image update
        rospy.Timer(rospy.Duration(0.1), self.update_callback)
        
        # self.init_trackers(None)
        
        
    def update_callback(self, event=None):
        """Callback to update the image on the camera topic."""
        
        ret, frame = self.video_cap.read()
        if not ret:
            rospy.logwarn(f"Couldn't get image frame for camera {self.cam_idx}")
            return
            
        # convert image color format and publish
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        image: Image = self.bridge.cv2_to_imgmsg(cvim=frame, encoding="rgb8")
        image.header.frame_id = f"{self.cam_idx}"
        self.image_pub.publish(image)
        
        
        # if self.trackers_running:
        #     self.update_trackers(frame)
            
        self.last_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
    def init_trackers(self, starting_pts: list):
        # TODO: temporary tracker initialization, change later using a proper system
        
        ret, frame = self.video_cap.read()
        if not ret:
            rospy.logwarn(f"Could not initialize tracking on camera {self.cam_idx}")
            return
        
        # init 1 tracked point
        plt.imshow(frame)
        pt = plt.ginput(1)
        plt.close()
        starting_pts = pt
        
        p0 = []
        for point in starting_pts: p0.append(list(point))
        self.last_points = np.array(p0, dtype = np.float32)
        self.last_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        self.trackers_running = True
        
    def update_trackers(self, frame):
        """Update trackers with a Lukas Kanade update given the new frame

        Args:
            frame (list): new image frame
        """
        
        # update our LK trackers
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        lk_params = dict(winSize = (32, 32), maxLevel = 8, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 9, 0.02555))
        p1, _, _ = cv2.calcOpticalFlowPyrLK(self.last_frame, gray_img, self.last_points, None, **lk_params)
        self.last_frame = gray_img.copy()
        self.last_points = p1.copy()
        
        marked_up_image = np.copy(frame)
        
        # publish tracked points as ROS msg.
        tracked_points = TrackedPoints()
        tracked_points.points = []
        for point in p1:
            tracked_pt: TrackedPoint = TrackedPoint()
            tracked_pt.x = point[0]
            tracked_pt.y = point[1]
            tracked_points.points.append(tracked_pt)
            cv2.circle(marked_up_image, (int(point[0]), int(point[1])), 0, color=(0, 255, 0), thickness=10)
            
        cv2.circle(marked_up_image, (self.target[0], self.target[1]), 0, color=(255, 0, 0), thickness=10)
        self.tracked_points_pub.publish(tracked_points)
        
        img_msg: Image = self.bridge.cv2_to_imgmsg(cvim=marked_up_image, encoding="rgb8")
        self.tracked_image_pub.publish(img_msg)
        
        error: Error = Error()
        error.error = [p1[0][0] - self.target[0], p1[0][1] - self.target[1]]
        self.image_error_pub.publish(error)

def main(args):
    rospy.sleep(3) # this node seems to need a sleep to start properly in tmux, not sure why, TODO: try to fix.
    rospy.loginfo("Starting a CameraNode ...")
    node = CameraNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down Camera...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)