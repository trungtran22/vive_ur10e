#!/usr/bin/env python3
import sys
import rospy
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped # <-- 1. THÊM TransformStamped
from std_msgs.msg import Float32
import moveit_commander
from math import tau
import tf2_ros
import tf2_geometry_msgs
# --- 2. THÊM Broadcaster TĨNH ---
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

# --- Class DataHolder (Không đổi) ---
class DataHolder:
    def __init__(self):
        self.pose_vr = PoseStamped()
        self.trigger = Float32()
        self.sub_pose = rospy.Subscriber("/vive_rightPose", PoseStamped, self.pose_vrCallback) # <-- Dùng topic của BẠN
        self.sub_trigger = rospy.Subscriber("/vive_rightTrigger", Float32, self.triggerCallback) # <-- Dùng topic của BẠN

    def pose_vrCallback(self, msg):
        self.pose_vr = msg
    def triggerCallback(self, msg):
        self.trigger = msg
# ---------------------------------

def go_to_initial_state():
        
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 4
        joint_goal[2] = tau / 4
        joint_goal[3] = -tau / 4
        joint_goal[4] = -tau / 4
        joint_goal[5] = -tau/6  # 1/6 of a turn
        move_group.go(joint_goal, wait=True)
        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()    
if __name__ == "__main__":
    rospy.init_node('vive_control', anonymous=True)
    
    # --- 3. ĐÂY LÀ PHẦN THAY THẾ CHO `rosrun tf` ---
    # Tạo một broadcaster (bộ phát) TĨNH
    broadcaster = StaticTransformBroadcaster()

    # Tạo một message Transform
    static_transform = TransformStamped()
    
    static_transform.header.stamp = rospy.Time.now()
    static_transform.header.frame_id = "controller_LHR_FDE6FBC3"      # <-- Gốc tọa độ VIVE (từ publisher C++)
    static_transform.child_frame_id = "base_link"  # <-- Gốc tọa độ Robot (từ MoveIt)
    
    # Chúng ta giả định 2 gốc này trùng nhau (x=0, y=0, z=0, quay=0)
    static_transform.transform.translation.x = -1.1
    static_transform.transform.translation.y = 0.28
    static_transform.transform.translation.z = 1.0
    static_transform.transform.rotation.x = 0.0
    static_transform.transform.rotation.y = 0.0
    static_transform.transform.rotation.z = 0.0
    static_transform.transform.rotation.w = 1.0

    # Gửi đi transform này (chỉ cần gửi 1 lần)
    broadcaster.sendTransform(static_transform)
    rospy.loginfo("Đã publish 'cầu nối' TF tĩnh (base -> base_link) vào code.")
    # --- KẾT THÚC PHẦN THAY THẾ ---

    # --- Code cũ của bạn (vẫn giữ nguyên) ---
    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    data = DataHolder()

    # Thiết lập TF2 Listener (để "nghe")
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)

    go_to_initial_state()
    rate = rospy.Rate(10) 

    rospy.loginfo("Sẵn sàng điều khiển. Hãy bóp cò VIVE.")

    while not rospy.is_shutdown():
        robot_base_frame = move_group.get_planning_frame() # (sẽ là "base_link")

        if data.trigger.data > 0.99:
            try:
                # Đảm bảo VIVE publish pose với frame_id là "base"
                if not data.pose_vr.header.frame_id:
                    data.pose_vr.header.frame_id = "base" # Gán frame_id nếu bị thiếu
                
                # Chờ transform (từ "base" sang "base_link")
                tf_buffer.can_transform(robot_base_frame, data.pose_vr.header.frame_id, rospy.Time(0), rospy.Duration(1.0))

                # Thực hiện transform
                target_pose_stamped = tf_buffer.transform(data.pose_vr, robot_base_frame, rospy.Duration(1.0))
                
                # --- GHI ĐÈ ORIENTATION (Nếu bạn muốn cố định hướng) ---
                target_pose_stamped.pose.orientation.x = 0.0
                target_pose_stamped.pose.orientation.y = 1.0
                target_pose_stamped.pose.orientation.z = 0.0
                target_pose_stamped.pose.orientation.w = 0.0
                # ----------------------------------------------------

                move_group.set_pose_target(target_pose_stamped)
                move_group.go(wait=True)
                move_group.stop()
                move_group.clear_pose_targets()
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                rospy.logwarn(f"Lỗi TF: {e}. Đang chờ transform từ '{data.pose_vr.header.frame_id}' đến '{robot_base_frame}'...")
                rospy.sleep(1.0)
        
        rate.sleep()