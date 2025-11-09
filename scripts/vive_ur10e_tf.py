#!/usr/bin/env python3
import sys
import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped
import moveit_commander
from math import tau, pi
import tf2_ros
import tf2_geometry_msgs
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from tf import transformations as tf_math


class VRDataHolder:
    def __init__(self):
        self.pose_vr = PoseStamped()
        self.trigger = Float32()
        self.sub_pose = rospy.Subscriber("/vive_rightPose", PoseStamped, self.pose_vrCallback)
        self.sub_trigger = rospy.Subscriber("/vive_rightTrigger", Float32, self.triggerCallback)

    def pose_vrCallback(self, msg):
        self.pose_vr = msg
    def triggerCallback(self, msg):
        self.trigger = msg

def go_to_initial_state(move_group):
    
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 4
    joint_goal[2] = tau / 4
    joint_goal[3] = -tau / 4
    joint_goal[4] = -tau / 4
    joint_goal[5] = -tau/6
    move_group.go(joint_goal, wait=True)
    move_group.stop()

# MAIN
def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('vive_control_tf2', anonymous=True)
    target_pose_pub = rospy.Publisher("/vive_target_pose", PoseStamped, queue_size=1)
    # ---Publish TF Bridge---
    rospy.loginfo("Publishing static TF transform (vive_world -> base_link)")
    broadcaster = StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "vive_world"        # <-- VIVE (Y-Up)
    t.child_frame_id = "base_link"          # <-- Robot (Z-Up)
    
    # ---CALIBRATION---
    t.transform.translation.x = -1.0 
    t.transform.translation.y = -1.0
    t.transform.translation.z = -0.8
    
    # Transform rotation Y to Z-Up
    q = tf_math.quaternion_from_euler(-pi/2, 0, 0) # (roll, pitch, yaw)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    
    broadcaster.sendTransform(t) 
    

    # INIT TF LISTENER
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    # MOVEIT INIT
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    data = VRDataHolder()
    robot_base_frame = move_group.get_planning_frame()
    go_to_initial_state(move_group)
    rate = rospy.Rate(10) 
    rospy.loginfo("Ready to control. Pull the trigger to move the robot.")

    
    while not rospy.is_shutdown():
        if data.trigger.data > 0.99:
            try:
                pose_in_vive_frame = data.pose_vr
                if not pose_in_vive_frame.header.frame_id:
                     pose_in_vive_frame.header.frame_id = "vive_world"
                
                target_pose_stamped = tf_buffer.transform(
                    pose_in_vive_frame,
                    robot_base_frame,
                    rospy.Duration(1.0)
                )

                # Hardcode orientation
                target_pose_stamped.pose.orientation.x = 0.0
                target_pose_stamped.pose.orientation.y = 1.0
                target_pose_stamped.pose.orientation.z = 0.0
                target_pose_stamped.pose.orientation.w = 0.0

                # Publish target pose for debugging
                target_pose_pub.publish(target_pose_stamped)
                # Move the robot
                move_group.set_pose_target(target_pose_stamped)
                move_group.go(wait=True)
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e}...")
                rospy.sleep(1.0)
            finally:
                move_group.stop()
                move_group.clear_pose_targets()
        
        rate.sleep()

    moveit_commander.roscpp_shutdown()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass