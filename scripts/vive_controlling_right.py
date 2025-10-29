import rospy
import moveit_commander
from moveit_msgs.msg import Constraints, JointConstraint
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
import tf2_ros
import tf2_geometry_msgs

# --- Simplified DataFlowManager ---
# Only subscribes to pose and trigger
class DataFlowManager:
    def __init__(self):
        self.sub_pose = None
        self.sub_trigger = None
        self.pub_processed = None
        self.pub_currentPose = None

    def init(self, node_handle, robot_arm_p):
        # Subscribe only to pose and trigger
        self.sub_pose = rospy.Subscriber("/vive_rightPose", PoseStamped, robot_arm_p.VR_poseControllerCallback)
        self.sub_trigger = rospy.Subscriber("/vive_rightTrigger", Float32, robot_arm_p.VR_triggerCallback)

        # Publishers remain useful for debugging
        self.pub_processed = rospy.Publisher("/vive/processed/pose", PoseStamped, queue_size=1)
        self.pub_currentPose = rospy.Publisher("/robot/currentPose", PoseStamped, queue_size=1)

# --- Simplified ControllerState ---
# Only holds pose and trigger data
class ControllerState:
    def __init__(self):
        self.pose = PoseStamped()
        self.trigger = 0.0  # Default to float
        self.startedTrackingPose = PoseStamped()
        self.startedTrackingPose.pose.orientation.w = 1

# --- Modified RobotArm ---
# Now holds the move_group and manages itself
class RobotArm:
    def __init__(self, move_group):
        self.endLinkName = None
        self.io = None
        self.orientationLock = False
        self.orientationLockPose = PoseStamped()
        self.targetPose = PoseStamped()
        self.controllerState = ControllerState()
        self.startedTrackingPose = PoseStamped()
        self.currentArmPose = PoseStamped()
        self.move_group_p = move_group  # Store the MoveGroupCommander

    def init(self, node_handle, end_link_name, controller_name):
        self.endLinkName = end_link_name
        self.io = DataFlowManager()
        self.io.init(controller_name, node_handle, self)

    def processOrientation(self, poseStamped):
        q_processed = tf2_ros.transformations.quaternion_from_euler(0, 0, 0)  # Placeholder for orientation processing
        poseStamped.pose.orientation = tf2_geometry_msgs.Quaternion(q_processed[0], q_processed[1], q_processed[2], q_processed[3])

    def processPosition(self, poseStamped):
        # Implement position processing
        pass

    def processPose(self):
        # Implement pose processing
        pass

    def VR_poseControllerCallback(self, msg):
        self.controllerState.pose = msg
        self.processPose()
        self.io.pub_processed.publish(self.targetPose)
        self.io.pub_currentPose.publish(self.move_group_p.getCurrentPose(self.endLinkName))

    def VR_triggerCallback(self, msg):
        self.controllerState.trigger = msg.data

    # Callbacks for menu and grip have been removed

    # --- Methods from old Robot class, now part of RobotArm ---
    def setPoseTarget(self):
        self.move_group_p.clearPoseTargets()
        self.move_group_p.setStartStateToCurrentState()
        self.move_group_p.setPoseTarget(self.targetPose.pose, self.endLinkName)

    def move(self):
        """Sets the pose target and executes the movement."""
        self.setPoseTarget()
        self.move_group_p.move()

    def setJointConstraints(self):
        jointConstraints = []
        jointNames = self.move_group_p.getJointNames()
        jointValues = self.move_group_p.getCurrentJointValues()
        for joint_name, joint_value in zip(jointNames, jointValues):
            current = JointConstraint()
            current.joint_name = joint_name
            current.position = joint_value
            current.tolerance_above = 100
            current.tolerance_below = 100
            jointConstraints.append(current)
        constraints = Constraints()
        constraints.joint_constraints = jointConstraints
        constraints.name = "joint_constraints"
        self.move_group_p.setPathConstraints(constraints)

# The Robot class (which was a list wrapper) has been removed

def main():
    rospy.init_node('moveit_controller')
    node_handle = rospy.get_namespace()
    
    # AsyncSpinner is still needed for MoveIt!
    spinner = rospy.AsyncSpinner(4)
    spinner.start()

    
    # --- Read simplified param data ---
    rightControllerName = rospy.get_param("~right_controller_name", "right")
    robotPlanningGroup = rospy.get_param("~robot_planning_group", "manipulator")
    rightEndLinkName = rospy.get_param("~right_end_link_name", "wrist_3_link")

    # Parameters for left controller and numberOfArms are removed
    rospy.loginfo(" Initializing for UR10e.")

    # --- Setup MoveIt ---
    move_group = moveit_commander.MoveGroupCommander(robotPlanningGroup)

    # --- Set up the single robot arm ---
    # We no longer need the Robot class wrapper
    right_arm = RobotArm(move_group)
    right_arm.init(node_handle, rightEndLinkName, rightControllerName)

    publish_frequency = 10
    loop_rate = rospy.Rate(publish_frequency)

    # --- Simplified main loop ---
    while not rospy.is_shutdown():
        # Check only the right arm's trigger
        if right_arm.controllerState.trigger > 0.99:
            rospy.loginfo("[ARM-MOVING]")
            right_arm.move()  # Call the arm's move method
        
        # We no longer call spinOnce() as AsyncSpinner handles it
        loop_rate.sleep()

if __name__ == '__main__':
    main()