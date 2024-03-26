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
    def __init__(self, learn_rate: float, step_size: float, max_it: int, threshold: int, num_joints: int):
        """Initialize visual servoing, setting up the required subscribers.

        Args:
            learn_rate (float): learning rate for broyden updates
            step_size (float): how much to step for joint movements
            max_it (int): max servo iterations
            threshold (int): error norm we consider to be "at the goal"
            num_joints (int): number of joints used
        """
        
        rospy.init_node("visual_servo")
        rospy.loginfo("Starting VisualServoing Node")
        
        self.kinova: KinovaGen3 = KinovaGen3()
        
        self.max_it = max_it
        self.threshold = threshold 
        self.step_size = step_size # TODO: increase this as we get closer?
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
        
        # Kinova has 7 joints.
        # For fine VS, we may want to use all 7; however generally we only want to use 4. Those are:
        #   - idx 0: rotation of the arm base
        #   - idx 1: rotation of 2nd arm link
        #   - idx 3: rotation of 3rd arm link
        #   - idx 5: rotation of 4th arm link
        # Anything more than this shouldn't be necessary for large, coarse motions
        self.joints = np.deg2rad(
            np.array(
                [-0.1336059570312672, -28.57940673828129, -179.4915313720703, -147.7, 0.06742369383573531, -57.420898437500036, 89.88030242919922]
                )
            )
        
        # NOTE: temp starting angles.
        self.joints = np.array([-0.19655300635664474, 0.8299573758083271, -2.8110765191316562, -1.0309490727538062, -0.31231795578129784, -1.1241426666756027, 1.7763640002834045])
        self.kinova.send_joint_angles(np.copy(self.joints))
        self.latest_joints = np.array([self.joints[0], self.joints[1], self.joints[3], self.joints[5]])
        
        # --- Subscribers ---
        self.start_subscriber = rospy.Subscriber("/vs_start", Empty, self.start_callback)
        
        self.error_subscriber = rospy.Subscriber("/image_error", Error, self.img_error_callback)
        self.eef_subscriber = rospy.Subscriber("/eef_pos", TrackedPoints, self.eef_pose_callback)
        
        rospy.loginfo("Waiting for messages on /image_error and /eef_pos")
        rospy.wait_for_message("/image_error", Error)
        rospy.wait_for_message("/eef_pos", TrackedPoints)
        
        # NOTE: we can get kinova joint info if we want to subscribe from /my_gen3/joint_states
        # - might be useful later.
        
        rospy.loginfo("Visual Servoing is ready to run!")
        
        
    def eef_pose_callback(self, data: TrackedPoints):
        """Callback for getting the latest eef pose

        Args:
            data (TrackedPoints): last eef pose
        """
        if len(data.points) == 0: return
        self.latest_eef = np.array([data.points[0].x, data.points[0].y])
        
    def img_error_callback(self, data: Error):
        """Callback for getting the latest image error

        Args:
            data (Error): last error
        """
        if len(data.error) == 0: return
        self.latest_error = np.asarray([data.error])
        
    def start_callback(self, data: Empty):
        """Callback for the start command

        Args:
            data (Empty): empty ros message
        """
        if not self.running:
            self.running = True
            self.visual_servo_converge(self.max_it, self.step_size)
            self.running = False
        
    def move(self, delta):
        """Move the wam simulation by delta

        Args:
            delta (list): delta for the 3dof
        """
        self.joints[0] += delta[0]
        self.joints[1] += delta[1]
        self.joints[3] += delta[2]
        self.joints[5] += delta[3]
        
        # NOTE: motion blocks!
        rospy.loginfo(f"Sending Joints: {self.joints}")
        self.kinova.send_joint_angles(np.copy(self.joints))
        rospy.loginfo("Joint Move Complete.")
        self.latest_joints = np.array([self.joints[0], self.joints[1], self.joints[3], self.joints[5]])
        
    def broyden_update(self):
        """Update Jacobian with a Broyden Update."""
        current_eef = self.latest_eef
        delta_eef = self.prev_eef - current_eef
        
        # if tracked change is too small, return
        if np.linalg.norm(delta_eef) < 10: return
        
        delta_x = self.latest_joints - self.prev_joints
        delta_e = (self.latest_error - self.prev_error)[0]
        
        # prevent divide by what is essentially 0 in the case of jitters in trackers, but the arm hasn't moved.
        if np.linalg.norm(delta_x) < 0.0001: return
        
        # broyden update
        numerator = np.subtract(delta_e.T, self.jacobian @ delta_x.T)
        update = (np.reshape(numerator, (numerator.size, 1)) @ np.reshape(delta_x, (1, delta_x.size))) / (delta_x @ delta_x.T)
        
        self.jacobian = self.jacobian + self.learn_rate * update
        self.inverse_jacobian = np.linalg.pinv(self.jacobian)
        
        self.prev_error = self.latest_error
        self.prev_eef = self.latest_eef
        self.prev_joints = self.latest_joints
    
    def update_jacobian(self):
        """Generate an initial jacobian, or update to reset it by doing basis movements."""
        self.jacobian = np.zeros((self.latest_error.size, self.num_joints))
        delta = 0.05
        
        # loop for each joint
        move = [0.0, 0.0, 0.0, 0.0]
        for i in range(self.num_joints):
            initial_error = self.latest_error
            
            # move robot
            move[i] = delta
            self.move(move)
            
            after_error = self.latest_error
            rospy.loginfo(f"ERROR BEFORE: {initial_error}")
            rospy.loginfo(f"ERROR AFTER: {after_error}")
            
            # move back
            move[i] = -delta
            self.move(move)
            
            self.jacobian[:, i] = (after_error - initial_error) / delta
            
            rospy.loginfo(f"JACOBIAN: {self.jacobian}")
            move = [0.0, 0.0, 0.0, 0.0]
        
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
    
    def visual_servo_converge(self, max_it: int, step_size: float):
        """Run visual servoing and attempt to converge

        Args:
            max_it (int): max iterations
            step_size (float): step size for updates
        """
        rospy.loginfo(f"Starting VS Converge with max iterations {max_it} and step_size {step_size}")
        self.update_jacobian()
        for i in range(max_it):
            self.broyden_update()
            
            delta_move = self.get_move()[0]
            rospy.loginfo(f"DELTA MOVE: {delta_move}")
            
            self.move(delta_move)
            
            if np.linalg.norm(self.latest_error) < self.threshold:
                rospy.loginfo(f"Servoing converged after {i} iterations")
                return
        
        rospy.logwarn(f"Servoing did not converge after {self.max_it} iterations.")
            

def main(args):
    rospy.sleep(3) # this node seems to need a sleep to start properly in tmux, not sure why, TODO: try to fix.
    rospy.loginfo("Starting vs node...")
    node = VisualServoing(learn_rate=0.2, step_size=0.1, max_it=100, threshold=5, num_joints=4)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down vs node...")
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)