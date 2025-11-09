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
import copy 

# --- Adjust this scale factor --- 
# 0.3 means 1 meter of Vive movement = 0.3 meter of robot movement
# 0.3 is for UR3e with small workspace, adjust as needed (0.5 for UR5e, 1.0 for UR10e)
SCALE_FACTOR = 0.3 
# ---------------------------------

# --- DataHolder with trigger state management ---
class DataHolder:
    def __init__(self):
        self.pose_vr = PoseStamped()
        self.trigger_value = 0.0
        self.is_pressed = False
        self.just_pressed = False # Flag for "just pressed"
        
        self.sub_pose = rospy.Subscriber("/vive_rightPose", PoseStamped, self.pose_vrCallback)
        self.sub_trigger = rospy.Subscriber("/vive_rightTrigger", Float32, self.triggerCallback)

    def pose_vrCallback(self, msg):
        self.pose_vr = msg
        
    def triggerCallback(self, msg):
        new_val = msg.data
        new_state = new_val > 0.99  

        if new_state and not self.is_pressed:
            self.just_pressed = True
        
        self.trigger_value = new_val
        self.is_pressed = new_state


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
    rospy.init_node('vive_control_tf2_relative', anonymous=True)

   
    broadcaster = StaticTransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "vive_world"
    t.child_frame_id = "base_link"
    
    
    t.transform.translation.x = 1.0 
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.8
    
    
    q = tf_math.quaternion_from_euler(-pi/2, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]
    broadcaster.sendTransform(t)
    
    
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    data = DataHolder()
    robot_base_frame = move_group.get_planning_frame()
    go_to_initial_state(move_group)
    rate = rospy.Rate(10) 
    
    
    start_robot_pose = PoseStamped() # Robot pose when first trigger pressed
    start_vive_pose = PoseStamped()  # Vive pose when first trigger pressed
    
    rospy.loginfo("Pull the VIVE trigger.")


    while not rospy.is_shutdown():
        # Get vive_world frame
        pose_in_vive_frame = data.pose_vr
        if not pose_in_vive_frame.header.frame_id:
            pose_in_vive_frame.header.frame_id = "vive_world"

        try:
            #Get current VIVE pose in robot base frame
            current_vive_tf_pose = tf_buffer.transform(
                pose_in_vive_frame,
                robot_base_frame,
                rospy.Duration(1.0)
            )

            # Check trigger state
            if data.is_pressed:
                # If it's the first press
                if data.just_pressed:
                    rospy.loginfo("--- Starting Relatively Control ---")
                    # Save starting poses
                    start_robot_pose = move_group.get_current_pose(move_group.get_end_effector_link())
                    start_vive_pose = copy.deepcopy(current_vive_tf_pose)
                    data.just_pressed = False # Erase flag
                
                # 4. If trigger is HELD
                # Calculate deltas - change in VIVE position since starting pose
                delta_x = (current_vive_tf_pose.pose.position.x - start_vive_pose.pose.position.x) * SCALE_FACTOR
                delta_y = (current_vive_tf_pose.pose.position.y - start_vive_pose.pose.position.y) * SCALE_FACTOR
                delta_z = (current_vive_tf_pose.pose.position.z - start_vive_pose.pose.position.z) * SCALE_FACTOR

                # Calculating new goal pose = start_robot_pose + delta (scaled)
                goal_pose = PoseStamped()
                goal_pose.header.frame_id = robot_base_frame
                goal_pose.pose.position.x = start_robot_pose.pose.position.x + delta_x
                goal_pose.pose.position.y = start_robot_pose.pose.position.y + delta_y
                goal_pose.pose.position.z = start_robot_pose.pose.position.z + delta_z
                
                
                goal_pose.pose.orientation.x = 0.0
                goal_pose.pose.orientation.y = 1.0
                goal_pose.pose.orientation.z = 0.0
                goal_pose.pose.orientation.w = 0.0
                
                # Send to Moveit
                move_group.set_pose_target(goal_pose)
                move_group.go(wait=True)
            
            else: # If trigger is NOT pressed
                if not data.just_pressed: 
                    rospy.loginfo("--- Stopping Relatively Control ---")
                    data.just_pressed = False # Reset flag
                    move_group.stop()
                    move_group.clear_pose_targets()

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF error: {e}...")
            rospy.sleep(1.0)
        
        rate.sleep()

    moveit_commander.roscpp_shutdown()

if __name__=="__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass