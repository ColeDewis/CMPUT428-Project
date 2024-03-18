import sys
import rospy
import cv2
import cv_bridge

import numpy as np

from kortex_bringup import KinovaGen3

from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Empty
from visual_servoing.msg import Error, TrackedPoints

class VisualServoing:
    """Uncalibrated Visual Servoing."""
    def __init__(self, learn_rate: float, step_size: float, timeout_sec: int, num_joints: int):
        """Initialize visual servoing, setting up the required subscribers.

        Args:
            learn_rate (float): learning rate for broyden updates
            step_size (float): how much to step for joint movements
            timeout_sec (int): timeout for servoing in seconds
            num_joints (int): number of joints used
        """
        
        rospy.init_node("visual_servo")
        rospy.loginfo("Starting VisualServoing Node")
        
        # TODO: integrate with Kinova
        # self.kinova: KinovaGen3 = KinovaGen3()
        
        self.timeout_sec = timeout_sec
        self.step_size = step_size
        self.learn_rate = learn_rate
        self.num_joints = num_joints
        
        # state variables
        self.latest_eef = None
        self.latest_error = None
        self.latest_joints = None
        self.jacobian = None
        self.inverse_jacobian = None
        self.running = False
        
        # holds last state
        self.prev_eef = None
        self.prev_joints = None
        self.prev_error = None
        
        # temp for WAM
        # TODO: update for use with Kinova
        self.joints = [0.5, 0.5, 0.5]
        
        # --- Subscribers ---
        self.start_subscriber = rospy.Subscriber("/vs_start", Empty, self.start_callback)
        
        # TODO: trackers
        self.error_subscriber = rospy.Subscriber("/image_error", Error, self.img_error_callback)
        self.eef_subscriber = rospy.Subscriber("/eef_pos", TrackedPoints, self.eef_pose_callback)
        self.joint_subscriber = rospy.Subscriber("/joint_states", JointState, self.joint_callback)
        
        
        # Temp publisher for WAM, TODO: remove later
        self.joint_state_pub = rospy.Publisher("/joint_states", JointState, queue_size=10)
        
        # timer for joint pub, temp for wam
        rospy.Timer(rospy.Duration(0.1), self.joint_pub)
        
        
    def eef_pose_callback(self, data: TrackedPoints):
        """Callback for getting the latest eef pose

        Args:
            data (TrackedPoints): last eef pose
        """
        self.latest_eef = np.array([data.points[0].x, data.points[0].y])
        
    def img_error_callback(self, data: Error):
        """Callback for getting the latest image error

        Args:
            data (Error): last error
        """
        self.latest_error = np.asarray([data.error])
        
    def start_callback(self, data: Empty):
        """Callback for the start command

        Args:
            data (Empty): empty ros message
        """
        if not self.running:
            self.running = True
            self.visual_servo_converge(self.timeout_sec, self.step_size)
            self.running = False
            
    def joint_callback(self, data: JointState):
        """Callback for joint state messages (temp for WAM)

        Args:
            data (JointState): message received
        """
        # hardcoded 3DOF which should be first 3 joints in our mapping for now
        self.latest_joints = np.array([[data.position[0], data.position[1], data.position[2]]])
        
    def joint_pub(self, event=None):
        robot_state = JointState()
        robot_state.name = ["q1", "q2", "q4", "q3", 
                            "bhand_j11_joint", "bhand_j21_joint", 
                            "bhand_j12_joint", "bhand_j22_joint", 
                            "bhand_j32_joint", "bhand_j13_joint", 
                            "bhand_j23_joint", "bhand_j33_joint"]
        robot_state.header.stamp = rospy.get_rostime()
        robot_state.header.frame_id = "";
        robot_state.position = []
        for i in range(3):
            robot_state.position.append(self.joints[i]);
        
        for i in range(9):
            robot_state.position.append(0);
            
        self.joint_state_pub.publish(robot_state);
        
    def move(self, delta):
        """Move the wam simulation by delta

        Args:
            delta (list): delta for the 3dof
        """
        # temp for WAM
        self.joints[0] += delta[0]
        self.joints[1] += delta[1]
        self.joints[2] += delta[2]
        
        # TODO: send to kinova. Need to figure out which DOF we want.
        
    def broyden_update(self):
        """Update Jacobian with a Broyden Update."""
        current_eef = self.latest_eef
        delta_eef = self.prev_eef - current_eef
        
        # if tracked change is too small, return
        if np.linalg.norm(delta_eef) < 10: return
        
        delta_x = self.latest_joints - self.prev_joints
        delta_e = self.latest_error - self.prev_error
        
        # prevent divide by what is essentially 0 in the case of jitters in trackers
        if np.linalg.norm(delta_x) < 0.0001: return
        
        # broyden update
        update = (np.subtract(delta_e.T, self.jacobian @ delta_x.T) @ delta_x) / (delta_x @ delta_x.T)
        
        self.jacobian = self.jacobian + self.learn_rate * update
        self.inverse_jacobian = np.linalg.pinv(self.jacobian)
        
        self.prev_error = self.latest_error
        self.prev_eef = self.latest_eef
        self.prev_joints = self.latest_joints
    
    def update_jacobian(self):
        """Generate an initial jacobian, or update to reset it by doing basis movements."""
        self.jacobian = np.zeros((self.latest_error.size, self.num_joints))
        delta = 0.5
        
        # loop for each joint
        move = [0, 0, 0]
        for i in range(self.num_joints):
            initial_error = self.latest_error
            
            # move robot
            move[i] = delta
            self.move(move)
            rospy.sleep(rospy.Duration(0.5))
            
            after_error = self.latest_error
            rospy.loginfo(f"ERROR BEFORE: {initial_error}")
            rospy.loginfo(f"ERROR AFTER: {after_error}")
            
            # move back
            move[i] = -delta
            self.move(move)
            rospy.sleep(rospy.Duration(0.5))
            
            self.jacobian[:, i] = (after_error - initial_error) / delta
            
            rospy.loginfo(f"JACOBIAN: {self.jacobian}")
            move = [0, 0, 0]
        
        self.inverse_jacobian = np.linalg.pinv(self.jacobian)
        
        self.prev_error = self.latest_error
        self.prev_eef = self.latest_eef
        self.prev_joints = self.latest_joints
        
    def get_move(self):
        """Get the delta for the next motion.

        Returns:
            list: delta joint movement
        """
        return -self.step_size * (self.inverse_jacobian @ self.latest_error.T).T
    
    def visual_servo_converge(self, timeout_sec: int, step_size: float):
        """Run visual servoing and attempt to converge

        Args:
            timeout_sec (int): timeout in seconds
            step_size (float): step size for updates
        """
        rospy.loginfo(f"Starting VS Converge with timeout {timeout_sec} and step_size {step_size}")
        self.update_jacobian()
        for i in range(timeout_sec * 10):
            self.broyden_update()
            
            delta_move = self.get_move()[0]
            rospy.loginfo(f"DELTA MOVE: {delta_move}")
            
            self.move(delta_move)
            rospy.sleep(rospy.Duration(0.1))
            

def main(args):
    rospy.loginfo("Starting vs node...")
    node = VisualServoing(0.05, 0.05, 10, 3)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down vs node...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)