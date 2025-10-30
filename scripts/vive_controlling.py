
import rospy
import moveit_commander
from moveit_msgs.msg import Constraints, JointConstraint
from std_msgs.msg import Float32, Int32
from geometry_msgs.msg import PoseStamped
import threading
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, Int32
import tf2_ros
import tf2_geometry_msgs
from moveit_msgs.msg import Constraints, JointConstraint
import moveit_commander


class DataFlowManager:
    def __init__(self):
        self.sub_pose = None
        self.sub_trigger = None
        self.sub_menu = None
        self.sub_grip = None
        self.pub_processed = None
        self.pub_currentPose = None

    def init(self, controller_name, node_handle, robot_arm_p):
        self.sub_pose = rospy.Subscriber("/vive/controller/" + controller_name + "/pose", PoseStamped, robot_arm_p.VR_poseControllerCallback)
        self.sub_trigger = rospy.Subscriber("/vive/controller/" + controller_name + "/trigger", Float32, robot_arm_p.VR_triggerCallback)
        self.sub_menu = rospy.Subscriber("/vive/controller/" + controller_name + "/buttons/menu", Int32, robot_arm_p.VR_menuCallback)
        self.sub_grip = rospy.Subscriber("/vive/controller/" + controller_name + "/buttons/grip", Int32, robot_arm_p.gripCallback)

        self.pub_processed = rospy.Publisher("/moveit_vive/" + controller_name + "/processed/pose", PoseStamped, queue_size=1)
        self.pub_currentPose = rospy.Publisher("/robot/" + controller_name + "/currentPose", PoseStamped, queue_size=1)


class RobotArm:
    def __init__(self):
        self.endLinkName = None
        self.io = None
        self.orientationLock = False
        self.orientationLockPose = PoseStamped()
        self.targetPose = PoseStamped()
        self.controllerState = ControllerState()
        self.startedTrackingPose = PoseStamped()
        self.currentArmPose = PoseStamped()
        self.menuStreak = 0
        self.fullRobot = None

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
        self.io.pub_currentPose.publish(self.fullRobot.move_group_p.getCurrentPose(self.endLinkName))

    def VR_triggerCallback(self, msg):
        self.controllerState.trigger = msg.data

    def VR_menuCallback(self, msg):
        self.controllerState.menu = msg.data
        if self.controllerState.menu:
            self.menuStreak += 1
            if self.menuStreak > 20:
                self.orientationLock = not self.orientationLock
                if self.orientationLock:
                    self.orientationLockPose = self.fullRobot.move_group_p.getCurrentPose(self.endLinkName).pose
                print("Orientation Lock", self.orientationLock)
                self.menuStreak = 0
        else:
            self.menuStreak = 0

    def gripCallback(self, msg):
        self.controllerState.grip = msg.data
        if self.controllerState.grip:
            self.fullRobot.move_group_p.stop()


class Robot:
    def __init__(self, nr_arms):
        self.robotArms = []
        for _ in range(nr_arms):
            arm = RobotArm()
            arm.fullRobot = self
            self.robotArms.append(arm)
        self.move_group_p = moveit_commander.MoveGroupCommander()

    def setPoseTargets(self):
        self.move_group_p.clearPoseTargets()
        self.move_group_p.setStartStateToCurrentState()
        for arm in self.robotArms:
            self.move_group_p.setPoseTarget(arm.targetPose.pose, arm.endLinkName)

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


class ControllerState:
    def __init__(self):
        self.pose = PoseStamped()
        self.trigger = 0
        self.menu = 0
        self.grip = 0
        self.startedTrackingPose = PoseStamped()
        self.startedTrackingPose.pose.orientation.w = 1  # Assuming identity quaternion

def main():
    rospy.init_node('moveit_controller')
    node_handle = rospy.get_namespace()
    
    # This is used to increase the number of threads used.
    # It needs more threads because otherwise it will fail to get
    # the current pose of the robot
    spinner = rospy.AsyncSpinner(4)
    spinner.start()

    
    # Read param data
    rightControllerName = rospy.get_param("~right_controller_name", "right")
    leftControllerName = rospy.get_param("~left_controller_name", "left")
    robotPlanningGroup = rospy.get_param("~robot_planning_group", "manipulator")
    rightEndLinkName = rospy.get_param("~right_end_link_name", "wrist_3_link")
    leftEndLinkName = rospy.get_param("~left_end_link_name", "wrist")
    numberOfArms = rospy.get_param("~number_of_arms", 2)
    rospy.loginfo("[MOVEIT_VIVE]: NUMBER OF ARMS: {}".format(numberOfArms))

    assert numberOfArms > 0
    controllerName = [rightControllerName, leftControllerName]
    endLinkName = [rightEndLinkName, leftEndLinkName]

    # Setup MoveIt
    move_group = moveit_commander.MoveGroupCommander(robotPlanningGroup)

    # Set up the robot arms (publishers and subscribers)
    robot = Robot(numberOfArms)
    for i in range(numberOfArms):
        robot.robotArms[i].init(node_handle, endLinkName[i], controllerName[i])

    robot.move_group_p = move_group

    publish_frequency = 10

    # Start running loop
    loop_rate = rospy.Rate(publish_frequency)
    while not rospy.is_shutdown():
        if any(arm.controllerState.trigger > 0.99 for arm in robot.robotArms):
            if any(not arm.controllerState.grip for arm in robot.robotArms):
                rospy.loginfo("[ARM-MOVING")
                robot.setPoseTargets()
                robot.move_group_p.move()
        rospy.spinOnce()
        loop_rate.sleep()

if __name__ == '__main__':
    main()
