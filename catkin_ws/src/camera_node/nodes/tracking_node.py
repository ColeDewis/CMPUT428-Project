import sys
import rospy
import cv2
import cv_bridge
import numpy as np
from random import randint

from sensor_msgs.msg import Image
from visual_servoing.msg import Error, TrackedPoint, TrackedPoints
from custom_msgs.msg import ErrorDefinition, TrackRequest, Point2D

import sympy as sy
import matplotlib.pyplot as plt

class ErrorInfo:
    LINETOLINE = "lnln"
    POINTTOPOINT = "ptpt"
    POINTTOLINE = "ptln"
    def __init__(self, err_type, formula, idxs, err_idx):
        self.err_type = err_type
        self.formula = formula
        self.idxs = idxs
        self.err_idx = err_idx
        
        # random color
        self.b = randint(0, 255)
        self.g = randint(0, 255)
        self.r = randint(0, 255)
        
    def draw(self, img, points):
        if self.err_type == ErrorInfo.LINETOLINE:
            p1 = list(map(int, [points[self.idxs[0]][0], points[self.idxs[0]][1]]))
            p2 = list(map(int, [points[self.idxs[1]][0], points[self.idxs[1]][1]]))
            p3 = list(map(int, [points[self.idxs[2]][0], points[self.idxs[2]][1]]))
            p4 = list(map(int, [points[self.idxs[3]][0], points[self.idxs[3]][1]]))
            
            cv2.line(img, p1, p2, color=(self.b, self.g, self.r), thickness=10)
            cv2.line(img, p3, p4, color=(self.b, self.g, self.r), thickness=10)
            
        elif self.err_type == ErrorInfo.POINTTOPOINT:
            p1 = list(map(int, [points[self.idxs[0]][0], points[self.idxs[0]][1]]))
            p2 = list(map(int, [points[self.idxs[1]][0], points[self.idxs[1]][1]]))
            
            cv2.circle(img, p1, 0, color=(self.b, self.g, self.r), thickness=10)
            cv2.circle(img, p2, 0, color=(self.b, self.g, self.r), thickness=10)
        
        elif self.err_type == ErrorInfo.POINTTOLINE:
            p1 = list(map(int, [points[self.idxs[0]][0], points[self.idxs[0]][1]]))
            p2 = list(map(int, [points[self.idxs[1]][0], points[self.idxs[1]][1]]))
            p3 = list(map(int, [points[self.idxs[2]][0], points[self.idxs[2]][1]]))
            
            cv2.circle(img, p1, 0, color=(self.b, self.g, self.r), thickness=10)
            cv2.line(img, p2, p3, color=(self.b, self.g, self.r), thickness=10)

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
        self.error_formulas = {}
        self.target = {}
        self.trackers_running = {}
        for idx in self.cam_indices:
            self.camera_subscribers[idx] = rospy.Subscriber(f"/cameras/cam{idx}", Image, self.update_trackers)
            self.tracking_publishers[idx] = rospy.Publisher(f"/cameras/cam{idx}/tracked_points", Image, queue_size=10)
            self.last_points[idx] = []
            self.error[idx] = []
            self.error_formulas[idx] = []
            self.last_frame[idx] = None
            self.target[idx] = None
            self.trackers_running[idx] = False
            
            rospy.loginfo(f"Waiting for message on /cameras/cam{idx}")
            rospy.wait_for_message(f"/cameras/cam{idx}", Image)
            
        self.bridge = cv_bridge.CvBridge()
        
        # --- Publishers ---
        self.tracked_points_pub = rospy.Publisher("/eef_pos", TrackedPoints, queue_size=10)
        
        self.image_error_pub = rospy.Publisher("/image_error", Error, queue_size=10)
        
        # timer for tracking + error update message publishing
        rospy.Timer(rospy.Duration(0.1), self.publish_tracking_and_error)
        
        # --- Subscribers ---
        self.error_request_subscriber = rospy.Subscriber("/tracking_node/error_request", ErrorDefinition, self.error_request_callback)
        
        rospy.loginfo("Tracking Node is ready!")
        
    def init_trackers(self, idx, starting_pts):
        """Initialize trackers for each camera."""
        # TODO: in the future, we probably will instead have another node (or something) to call a "task initialization" 
        
        # TODO: we will receive a TrackRequest which tells us points to track and whether its a line or a point - we can then use this to map to indexes
        # TODO: receive an ErrorDefinition which tells us ids associated with a certain definition
        rospy.loginfo("Tracker Init - waiting for msg")
        starting_frame = rospy.wait_for_message(f"/cameras/cam{idx}", Image)
        starting_frame = self.bridge.imgmsg_to_cv2(img_msg=starting_frame, desired_encoding="rgb8")
        
        # init 1 tracked point
        # plt.imshow(starting_frame)
        # pt = plt.ginput(2)
        # plt.close()
        # starting_pts = [pt[0]]
        # self.target[idx] = np.array([pt[1][0], pt[1][1]])
        
        p0 = []
        for point in starting_pts: p0.append(list(point))
        
        # add the points 
        if len(self.last_points[idx]) == 0:
            self.last_points[idx] = np.array(p0, dtype = np.float32)
            indexes = list(range(len(self.last_points[idx])))
        else:
            old_len = len(self.last_points[idx])
            self.last_points[idx] = np.vstack([self.last_points[idx], np.array(p0, dtype=np.float32)])
            indexes = list(range(old_len, len(self.last_points[idx])))
            
        self.last_frame[idx] = cv2.cvtColor(starting_frame, cv2.COLOR_BGR2GRAY)
        self.trackers_running[idx] = True
        err_idx = len(self.error[idx])
        self.error[idx].append([])
        
        return indexes, err_idx
        
    def error_request_callback(self, data: ErrorDefinition):
        """Callback when we receive a new error request message

        Args:
            data (ErrorDefinition): error definition to implement
        """
        rospy.loginfo("Error Request Init")
        cam_idx = data.cam_idx
        err_type = data.type
        components = data.components
        
        if err_type == "ptpt":
            # assume we have 2 points
            p1 = [components[0].points[0].x, components[0].points[0].y]
            p2 = [components[1].points[0].x, components[1].points[0].y]
            
            # develop the equation for error
            x1, y1, x2, y2 = sy.symbols("x1 y1 x2 y2")
            xy1 = sy.Matrix([x1, y1, 1])
            xy2 = sy.Matrix([x2, y2, 1])
            error = xy1.cross(xy2)
            indexes, err_idx = self.init_trackers(cam_idx, [p1, p2])
            
        elif err_type == "ptln":
            # we don't know what is what so check
            if components[0].type in ("fp", "tp"):
                # first one is point
                pt = [components[0].points[0].x, components[0].points[0].y]
                lp1 = [components[1].points[0].x, components[1].points[0].y]
                lp2 = [components[1].points[1].x, components[1].points[1].y]
            else:
                # second one is point
                pt = [components[1].points[0].x, components[1].points[0].y]
                lp1 = [components[0].points[0].x, components[0].points[0].y]
                lp2 = [components[0].points[1].x, components[0].points[1].y]
            
            x1, y1, x2, y2, x3, y3 = sy.symbols("x1 y1 x2 y2 x3 y3")
            xy1 = sy.Matrix([x1, y1, 1])
            xy2 = sy.Matrix([x2, y2, 1])
            xy3 = sy.Matrix([x3, y3, 1])
            error = xy1.dot(xy2.cross(xy3))
            indexes, err_idx = self.init_trackers(cam_idx, [pt, lp1, lp2])
            
        elif err_type == "lnln":
            # assume we have 2 lines
            p1 = [components[0].points[0].x, components[0].points[0].y]
            p2 = [components[0].points[1].x, components[0].points[1].y]
            p3 = [components[1].points[0].x, components[1].points[0].y]
            p4 = [components[1].points[1].x, components[1].points[1].y]
            x1, y1, x2, y2, x3, y3, x4, y4 = sy.symbols("x1 y1 x2 y2 x3 y3 x4 y4")
            xy1 = sy.Matrix([x1, y1, 1])
            xy2 = sy.Matrix([x2, y2, 1])
            xy3 = sy.Matrix([x3, y3, 1])
            xy4 = sy.Matrix([x4, y4, 1])
            l2 = xy3.cross(xy4)
            error = xy1.dot(l2) + xy2.dot(l2)
            indexes, err_idx = self.init_trackers(cam_idx, [p1, p2, p3, p4])
        
        self.error_formulas[cam_idx].append(ErrorInfo(err_type, error, indexes, err_idx))
        rospy.loginfo(f"INDEXES: {indexes}")
        
    def update_trackers(self, data: Image):
        """Update trackers with a Lukas Kanade update given the new frame

        Args:
            data (Image): new ROS image message
        """
        camera_idx = int(data.header.frame_id)
        if not self.trackers_running[camera_idx]: return
        
        frame = self.bridge.imgmsg_to_cv2(img_msg=data, desired_encoding="rgb8")
        
        # update our LK trackers
        gray_img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        lk_params = dict(winSize = (32, 32), maxLevel = 8, criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 9, 0.02555))
        p1, _, _ = cv2.calcOpticalFlowPyrLK(self.last_frame[camera_idx], gray_img, self.last_points[camera_idx], None, **lk_params)
        self.last_frame[camera_idx] = gray_img.copy()
        self.last_points[camera_idx] = p1.copy()
        
        marked_up_image = np.copy(frame)
        
        # self.error[camera_idx] = []
        formula: ErrorInfo
        for formula in self.error_formulas[camera_idx]:
            args = {}
            for i, idx in enumerate(formula.idxs):
                args["x"+str(i+1)] = p1[idx][0]
                args["y"+str(i+1)] = p1[idx][1]
            
            # duck typing to convert to a list for convenience
            err = formula.formula.evalf(subs=args) # eval error with sympy
            try:
                err = list(err)
            except TypeError:
                err = [err]
                
            formula.draw(marked_up_image, p1)
                
            rospy.loginfo(f"{err}")
            self.error[camera_idx][formula.err_idx] = err
            # for e in err:
            #     self.error[camera_idx].append(e)
                
        img_msg: Image = self.bridge.cv2_to_imgmsg(cvim=marked_up_image, encoding="rgb8")
        self.tracking_publishers[camera_idx].publish(img_msg)        
        
        # self.publish_tracking_and_error()
                
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
            for err in val:
                for e in err:
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