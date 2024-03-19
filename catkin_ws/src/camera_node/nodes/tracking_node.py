import sys
import rospy
import cv2
import cv_bridge
import numpy as np

from sensor_msgs.msg import Image
from visual_servoing.msg import Error, TrackedPoint, TrackedPoints

import matplotlib.pyplot as plt

class TrackingNode:
    """Basic Node that runs a camera on a given camera index and publishes to ros."""
    def __init__(self):
        """Initialize a camera node by getting the rosparam and opening the opencv stream."""
        
        rospy.init_node("tracking_node")
        index_list_param = rospy.search_param("idxs")
        self.cam_indices = rospy.get_param(index_list_param, None)
        
        if self.cam_indices is None:
            rospy.logwarn("Must pass camera indexes as _idxs:=\"[idx1, idx2, ...]\"")
            exit()
    
        rospy.delete_param(index_list_param) # delete param so its needed for future runs.
        rospy.loginfo(f"Initialized TrackingNode with camera indexes {self.cam_indices}")
        
        self.trackers_running = False
        self.tracking_publishers = {}
        self.camera_subscribers = {}
        self.last_points = {}
        self.last_frame = {}
        self.error = {}
        self.target = {}
        for idx in self.cam_indices:
            self.camera_subscribers[idx] = rospy.Subscriber(f"/cameras/cam{idx}", Image, self.update_trackers)
            self.tracking_publishers[idx] = rospy.Publisher(f"/cameras/cam{idx}/tracked_points", Image, queue_size=10)
            self.last_points[idx] = []
            self.error[idx] = []
            self.last_frame[idx] = None
            self.target[idx] = None
            
            rospy.loginfo(f"Waiting for message on /cameras/cam{idx}")
            rospy.wait_for_message(f"/cameras/cam{idx}", Image)
            
        self.bridge = cv_bridge.CvBridge()
        
        
        # self.target = np.array([520, 170])
        
        # --- Publishers ---
        self.tracked_points_pub = rospy.Publisher("/eef_pos", TrackedPoints, queue_size=10)
        
        self.image_error_pub = rospy.Publisher("/image_error", Error, queue_size=10)
        
        # timer for tracking + error update message publishing
        rospy.Timer(rospy.Duration(0.1), self.publish_tracking_and_error)
        
        self.init_trackers()
        
    def init_trackers(self):
        """Initialize trackers for each camera."""
        # TODO: in the future, we probably will instead have another node (or something) to call a "task initialization" 
        for idx in self.cam_indices:
            starting_frame = rospy.wait_for_message(f"/cameras/cam{idx}", Image)
            starting_frame = self.bridge.imgmsg_to_cv2(img_msg=starting_frame, desired_encoding="rgb8")
            
            # init 1 tracked point
            plt.imshow(starting_frame)
            pt = plt.ginput(2)
            plt.close()
            starting_pts = [pt[0]]
            self.target[idx] = np.array([pt[1][0], pt[1][1]])
            
            p0 = []
            for point in starting_pts: p0.append(list(point))
            self.last_points[idx] = np.array(p0, dtype = np.float32)
            self.last_frame[idx] = cv2.cvtColor(starting_frame, cv2.COLOR_BGR2GRAY)
            
        self.trackers_running = True
        
    def update_trackers(self, data: Image):
        """Update trackers with a Lukas Kanade update given the new frame

        Args:
            data (Image): new ROS image message
        """
        if not self.trackers_running: return
        
        camera_idx = int(data.header.frame_id)
        frame = self.bridge.imgmsg_to_cv2(img_msg=data, desired_encoding="rgb8")
        
        # update our LK trackers
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        lk_params = dict(winSize = (32, 32), maxLevel = 8, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 9, 0.02555))
        p1, _, _ = cv2.calcOpticalFlowPyrLK(self.last_frame[camera_idx], gray_img, self.last_points[camera_idx], None, **lk_params)
        self.last_frame[camera_idx] = gray_img.copy()
        self.last_points[camera_idx] = p1.copy()
        
        marked_up_image = np.copy(frame)
        
        # draw points on image
        for point in p1:
            cv2.circle(marked_up_image, (int(point[0]), int(point[1])), 0, color=(0, 255, 0), thickness=10)
            
        # draw target, TODO: update logic
        cv2.circle(marked_up_image, (int(self.target[camera_idx][0]), int(self.target[camera_idx][1])), 0, color=(255, 0, 0), thickness=10)
        
        img_msg: Image = self.bridge.cv2_to_imgmsg(cvim=marked_up_image, encoding="rgb8")
        self.tracking_publishers[camera_idx].publish(img_msg)
        
        # TODO: temp error logic
        self.error[camera_idx] = [p1[0][0] - self.target[camera_idx][0], p1[0][1] - self.target[camera_idx][1]]
        
    def publish_tracking_and_error(self, event=None):
        """Publish tracked points and error from ALL cameras combined."""
        
        # get tracked points for all cameras, aggregate, and publish
        tracked_points: TrackedPoints = TrackedPoints()
        tracked_points.points = []
        for val in self.last_points.values():
            for pt in val:
                point = TrackedPoint()
                point.x = pt[0]
                point.y = pt[1]
                tracked_points.points.append(point)
        self.tracked_points_pub.publish(tracked_points)
        
        # get error for all cameras, aggregate, and publish
        error: Error = Error()
        error.error = []
        for val in self.error.values():
            for e in val:
                error.error.append(e)
        self.image_error_pub.publish(error)

def main(args):
    rospy.sleep(3) # this node seems to need a sleep to start properly in tmux, not sure why, TODO: try to fix.
    rospy.loginfo("Starting TrackingNode ...")
    node = TrackingNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down TrackingNode...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)