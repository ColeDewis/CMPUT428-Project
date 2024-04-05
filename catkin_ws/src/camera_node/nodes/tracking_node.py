import sys
import rospy
import cv2
import cv_bridge
import numpy as np
from random import randint

from sensor_msgs.msg import Image
from visual_servoing.msg import Error, TrackedPoint, TrackedPoints
from custom_msgs.msg import ErrorDefinition, TrackRequest, Point2D, DistanceDefinition

import sympy as sy
import matplotlib.pyplot as plt

class ErrorInfo:
    """Class representing an error constraint"""
    LINETOLINE = "lnln"
    POINTTOPOINT = "ptpt"
    POINTTOLINE = "ptln"
    def __init__(self, err_type, formula, idxs, err_idx, fixed_idxs, dist_ref_pts = None):
        """Initialize the constraint

        Args:
            err_type (str): type of constraint
            formula (sympy formula): sympy formula that defines the constraint
            idxs (list): list of indices in the tracking that correspond to the points used
            err_idx (int): index in the error vector that the error value goes into
            fixed_idxs (list): list of indices that are fixed points
        """
        self.err_type = err_type
        self.formula = formula
        self.idxs = idxs
        self.err_idx = err_idx
        self.fixed_idxs = fixed_idxs
        self.dist_ref_pts = dist_ref_pts
        
        # random color
        self.b = randint(0, 255)
        self.g = randint(0, 255)
        self.r = randint(0, 255)
        
    def draw(self, img, points):
        """Draws the error constraint

        Args:
            img (array): image array to drawit on (opencv)
            points (array): points from tracking to draw
        """
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
            
        if self.dist_ref_pts is not None:
            for pt in self.dist_ref_pts:
                rospy.loginfo(f"POINT {pt}")
                cv2.circle(img, (int(pt[0]), int(pt[1])), 5, (255, 0, 0), 3)
                

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
        rospy.loginfo("Tracker Init - waiting for msg")
        starting_frame = rospy.wait_for_message(f"/cameras/cam{idx}", Image)
        starting_frame = self.bridge.imgmsg_to_cv2(img_msg=starting_frame, desired_encoding="rgb8")
        
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
        
        # TEMP: ignore certain indexes
        # if cam_idx != 0: return
        
        err_type = data.type
        components = data.components
        distance_definition: DistanceDefinition = data.distance_info
        dist_valid = len(distance_definition.plane_points) == 4 and distance_definition.desired_distance != 0
        
        if err_type == "ptpt":
            # assume we have 2 points
            p1 = [components[0].points[0].x, components[0].points[0].y]
            p2 = [components[1].points[0].x, components[1].points[0].y]
            
            # develop the equation for error
            x1, y1, x2, y2 = sy.symbols("x1 y1 x2 y2")
            xy1 = sy.Matrix([x1, y1, 1])
            xy2 = sy.Matrix([x2, y2, 1])
            error = xy1.cross(xy2)[:2, :]
            dist_ref_pts = None
            
            # assume one of 2 points is fixed for simplicity for now.
            if components[0].type == "fp":
                if dist_valid:
                    dist_ref_pts = [np.copy(p1)]
                    p1 = self.update_targets_with_distance(distance_definition, [p1])[0]
                indexes, err_idx = self.init_trackers(cam_idx, [p1, p2])
                fixed_idxs = [indexes[0]]
            else:
                if dist_valid:
                    dist_ref_pts = [np.copy(p2)]
                    p2 = self.update_targets_with_distance(distance_definition, [p2])[0]
                indexes, err_idx = self.init_trackers(cam_idx, [p1, p2])
                fixed_idxs = [indexes[1]]
                
            
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
            
            # assume either point or line is fixed for simplicity for now
            if components[0].type == "fp" or components[1].type == "fp":
                if dist_valid:
                    dist_ref_pts = [np.copy(pt)]
                    pt = self.update_targets_with_distance(distance_definition, [pt])[0]
                indexes, err_idx = self.init_trackers(cam_idx, [pt, lp1, lp2])
                fixed_idxs = [indexes[0]]
            else:
                if dist_valid:
                    dist_ref_pts = [np.copy(lp1), np.copy(lp2)]
                    lp1, lp2 = self.update_targets_with_distance(distance_definition, [lp1, lp2])
                indexes, err_idx = self.init_trackers(cam_idx, [pt, lp1, lp2])
                fixed_idxs = [indexes[1], indexes[2]]
                
            
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
            
            # assume one line is fixed for simplicity for now
            if components[0].type == "fl":
                if dist_valid:
                    dist_ref_pts = [np.copy(p1), np.copy(p2)]
                    p1, p2 = self.update_targets_with_distance(distance_definition, [p1, p2])
                indexes, err_idx = self.init_trackers(cam_idx, [p1, p2, p3, p4])
                fixed_idxs = [indexes[0], indexes[1]]
            else:
                if dist_valid:
                    dist_ref_pts = [np.copy(p3), np.copy(p4)]
                    p3, p4 = self.update_targets_with_distance(distance_definition, [p3, p4])
                indexes, err_idx = self.init_trackers(cam_idx, [p1, p2, p3, p4])
                fixed_idxs = [indexes[2], indexes[3]]
                
        
        self.error_formulas[cam_idx].append(ErrorInfo(err_type, error, indexes, err_idx, fixed_idxs, dist_ref_pts))
        rospy.loginfo(f"INDEXES: {indexes}")
        
    def update_targets_with_distance(self, distance_info: DistanceDefinition, static_pts):
        """Calculates and moves the points given by static points to the positions defined by distance_info

        Args:
            distance_info (DistanceDefinition): object that gives distance definition information
            static_pts (list): fixed points we want to move

        Returns:
            list: new points
        """
        direction = distance_info.direction
        p1, p2, p3, p4 = distance_info.plane_points
        
        # We make the assumption that the points defined clockwise
        p1 = np.array([p1.x, p1.y, 1])
        p2 = np.array([p2.x, p2.y, 1])
        p3 = np.array([p3.x, p3.y, 1])
        p4 = np.array([p4.x, p4.y, 1])
        
        # calculate vanishing point 1
        l1 = np.cross(p1, p2)
        l2 = np.cross(p3, p4)
        v1 = np.cross(l1, l2) 
        v1 = v1 / v1[2]
        
        # calculate vanishing point 2
        l3 = np.cross(p1, p4)
        l4 = np.cross(p2, p3)
        v2 = np.cross(l3, l4)
        v2 = v2 / v2[2]
        
        # vanishing line
        vl = np.cross(v1, v2)
        
        new_pts = []
        # for each point, we have to extend it out
        for point in static_pts:
            a1 = np.array([point[0], point[1], 1])
            if distance_info.direction in (0, 2):
                # get reference distance in the direction we want
                
                # calculate p_t based on parallel assumption
                D = distance_info.reference_distance_u
                search_line = np.cross(a1, v1)
                v3_finder = np.cross(p4, a1)
                v3 = np.cross(v3_finder, vl)
                pt_finder = np.cross(p3, v3)
                p_t = np.cross(search_line, pt_finder)
                p_t = p_t / p_t[2]
                
                # set the proper vanishing point
                v = v1
                
            elif distance_info.direction in (1, 3):
                # get deference distance in the direction we want 
                D = distance_info.reference_distance_v
                
                # calculate p_t based on parallel assumption
                search_line = np.cross(a1, v2)
                v3_finder = np.cross(p4, a1)
                v3 = np.cross(v3_finder, vl)
                pt_finder = np.cross(p1, v3)
                p_t = np.cross(search_line, pt_finder)
                p_t = p_t / p_t[2]
                
                # set the proper vanishing point
                v = v2
                
            # NOW: segment a-p_t is the same as our reference distance p3-p4.
            # we have formulated our problem now as a search along line search_line, where our points that we use are:
            # 1) a, 2) p_t, 3) a+epsilon, 4) p*, 5) v1/v2
            # a - reference point for constraint
            # p_t =known
            # a + epsiolon known
            # p* we solve for
            # v1/v2 is vanishing point of parallel defined lines
            epsilon = 0.01 # epsilon is a distance along our line we define a point away from a1, so we have a 5th point for our math. 
            # TODO: we may not need epsilon, do some testing?
            
            # determine cr1
            d_a1_pt = np.linalg.norm(a1[:2] - p_t[:2])
            d_aeps_v1 = np.linalg.norm(a1[:2] - v[:2]) + epsilon # increase distance by epsilon to avoid zeros
            d_pt_aeps = np.linalg.norm(a1[:2] - p_t[:2]) + epsilon
            d_a1_v1 = np.linalg.norm(a1[:2] - v[:2])
            cr1 = (d_a1_pt * d_aeps_v1) / (d_pt_aeps * d_a1_v1)
            
            # determine distances related to cr2
            d_a1_s2 = np.linalg.norm(a1[:2] - p_t[:2])
            d_pt_v1 = np.linalg.norm(p_t[:2] - v[:2])
            
            d_star = distance_info.desired_distance
            
            rospy.loginfo(f"{D} - {cr1} - {d_pt_v1}")
            D_star = (d_star * d_a1_s2) / ((D / cr1) * d_pt_v1)
            p = (d_a1_v1 * D_star) / (D_star + 1)
            
            # get the unit vector between the point "a" and point "v"
            unit_vec = [(v[0] - a1[0])/d_aeps_v1, (v[1] - a1[1])/d_aeps_v1]
            
            # depending on the direction, we now need to move along the vector.
            # however, the direction depends on a) what direction we want and b) which direction the line towards
            # the vanishing point actually leads us
            if direction == 0: # positive x
                if v1[0] > a1[0]: # vanishing point goes right, follow it
                    p_img = [a1[0] + unit_vec[0] * p, a1[1] + unit_vec[1] * p]
                else: # vanishing point goes left, we want positive, so we go negative.
                    p_img = [a1[0] - unit_vec[0] * p, a1[1] - unit_vec[1] * p]
            elif direction == 1: # positive y
                if v2[1] > a1[1]: # vanishing point down, don't follow
                    p_img = [a1[0] - unit_vec[0] * p, a1[1] - unit_vec[1] * p]
                else: # vanishing point up, follow
                    p_img = [a1[0] + unit_vec[0] * p, a1[1] + unit_vec[1] * p]
            elif direction == 2: # negative x
                if v1[0] > a1[0]: # vanishing point left, want positive, so go right
                    p_img = [a1[0] - unit_vec[0] * p, a1[1] - unit_vec[1] * p]
                else: # vanishing point goes left, we follow it
                    p_img = [a1[0] + unit_vec[0] * p, a1[1] + unit_vec[1] * p]
            elif direction == 3: # negative y
                if v2[1] > a1[1]: # vanishing point down, follow
                    p_img = [a1[0] + unit_vec[0] * p, a1[1] + unit_vec[1] * p]
                else: # vanishing point up, don't follow
                    p_img = [a1[0] - unit_vec[0] * p, a1[1] - unit_vec[1] * p]
            
            new_pts.append(p_img)
            rospy.loginfo(f"OLD: {point}, NEW: {p_img}")
            
        return np.asarray(new_pts)
        
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
        
        marked_up_image = np.copy(frame)
        
        formula: ErrorInfo
        for formula in self.error_formulas[camera_idx]:
            # for all fixed indexes set them back to old value 
            for idx in formula.fixed_idxs:
                p1[idx] = self.last_points[camera_idx][idx]
            
            # setup args for sympy evaluation
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
                
            # rospy.loginfo(f"{err}")
            self.error[camera_idx][formula.err_idx] = err
                
        self.last_points[camera_idx] = p1.copy()
        img_msg: Image = self.bridge.cv2_to_imgmsg(cvim=marked_up_image, encoding="rgb8")
        self.tracking_publishers[camera_idx].publish(img_msg)        
        
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
    rospy.sleep(5) # this node seems to need a sleep to start properly in tmux, not sure why, TODO: try to fix.
    rospy.loginfo("Starting TrackingNode ...")
    node = TrackingNode()
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down TrackingNode...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)