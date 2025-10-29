// Written by Jonathan Österberg. This is the file that is added to the vive_ros package.
// If questions, contact me on jonte_ost@live.com

// Original code by Jonathan Österberg, simplified to only publish
// right controller pose and trigger.

#include <iostream>
#include "vive_pose.h" // Assumed to define VR_ControlHandler, VRC_RIGHT, etc.
#include "ros/ros.h"
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32.h>
#include <openvr.h>
#include <cmath> // For fmax, sqrt, copysign

using namespace vr;

// Global VR system pointer
IVRSystem* vr_pointer = NULL;


//  * @brief Remaps the pose from SteamVR coordinates to a different frame.
//  * @param pose The input pose from SteamVR.
//  * @return The remapped pose.

geometry_msgs::Pose Remap(geometry_msgs::Pose pose)
{
    double x = pose.position.x;
    double y = pose.position.y;
    double z = pose.position.z;

    pose.position.x = -z;
    pose.position.y = -x;
    pose.position.z = y;

    return pose;
}

/*
 * @brief Creates a geometry_msgs::PoseStamped message from OpenVR pose data.
 * @param controller_position The position vector.
 * @param controller_orientation The orientation quaternion.
 * @param frame_id The ROS frame ID for the header.
 * @return A populated PoseStamped message.
 */
geometry_msgs::PoseStamped MakeGeometryMsgFromData(HmdVector3_t controller_position, HmdQuaternion_t controller_orientation, const char* frame_id)
{
    geometry_msgs::PoseStamped controller_msg;
    controller_msg.header.frame_id = frame_id;
    controller_msg.pose.position.x = controller_position.v[0];
    controller_msg.pose.position.y = controller_position.v[1];
    controller_msg.pose.position.z = controller_position.v[2];

    // Apply the coordinate remapping
    controller_msg.pose = Remap(controller_msg.pose);

    // Remap orientation from SteamVR to the new frame
    controller_msg.pose.orientation.w = controller_orientation.w;
    controller_msg.pose.orientation.x = -controller_orientation.z;
    controller_msg.pose.orientation.y = -controller_orientation.x;
    controller_msg.pose.orientation.z = controller_orientation.y; 

    return controller_msg;
}

/*
 * Extracts the rotation quaternion from an OpenVR 3x4 matrix.
 */
HmdQuaternion_t GetRotation(HmdMatrix34_t matrix) 
{
    HmdQuaternion_t q;

    q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
    q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
    q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
    q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
    q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);

    return q;
}

/*
 * Extracts the position vector from an OpenVR 3x4 matrix.
 */
HmdVector3_t GetPosition(HmdMatrix34_t matrix) 
{
    HmdVector3_t vector;
    vector.v[0] = matrix.m[0][3];
    vector.v[1] = matrix.m[1][3];
    vector.v[2] = matrix.m[2][3];
    return vector;
}

/*
 * Polls OpenVR for the right controller's state and updates the handler.
 * controlHandler The controller data structure to update.
 */
void GetControllerState(VR_ControlHandler &controlHandler)
{   
    for (unsigned int deviceId = 0; deviceId < k_unMaxTrackedDeviceCount; deviceId++) {

        if (!vr_pointer->IsTrackedDeviceConnected(deviceId))
            continue;

        ETrackedDeviceClass cl = vr_pointer->GetTrackedDeviceClass(deviceId);

        // We are only interested in controllers
        if (cl == ETrackedDeviceClass::TrackedDeviceClass_Controller) {
            
            ETrackedControllerRole role = vr_pointer->GetControllerRoleForTrackedDeviceIndex(deviceId);

            // only interested in the RIGHT controller
            if (role == TrackedControllerRole_RightHand)
            {
                TrackedDevicePose_t trackedDevicePose;
                VRControllerState_t controllerState;

                // Get the controller's state and pose
                vr_pointer->GetControllerStateWithPose(TrackingUniverseStanding, deviceId, &controllerState, sizeof(controllerState), &trackedDevicePose);
                
                // Update the status (is the pose valid?)
                controlHandler.pController[VRC_RIGHT]->status = trackedDevicePose.bPoseIsValid;

                // Update trigger button value (analog axis 1)
                controlHandler.pController[VRC_RIGHT]->buttons.trigger = controllerState.rAxis[1].x;

                // If the pose is valid, update the pose message
                if(trackedDevicePose.bPoseIsValid)
                {
                    HmdVector3_t controller_position = GetPosition(trackedDevicePose.mDeviceToAbsoluteTracking);
                    HmdQuaternion_t controller_orientation = GetRotation(trackedDevicePose.mDeviceToAbsoluteTracking);
                    
                    // Use "base" as the frame_id, as in the original code
                    controlHandler.pController[VRC_RIGHT]->pose.msg = MakeGeometryMsgFromData(controller_position, controller_orientation, "base");
                }
                
                // Since we found the right controller, we can stop looping
                break; 
            }
        }  
    }
} 

/*
 * @brief Shuts down the OpenVR system.
 */
void Shutdown() 
{
    if (vr_pointer != NULL)
    {
        VR_Shutdown(); 
        vr_pointer = NULL;
    }
}

// --- Main Program ---
int main(int argc, char *argv[]) 
{
    // --- Initialize OpenVR ---
    EVRInitError eError = VRInitError_None;
    vr_pointer = VR_Init(&eError, VRApplication_Background);
    if (eError != VRInitError_None)
    {
        vr_pointer = NULL;
        printf("Unable to init VR runtime: %s \n", 
            VR_GetVRInitErrorAsEnglishDescription(eError));
        exit(EXIT_FAILURE);
    }

    // --- Initialize ROS ---
    ros::init(argc, argv, "vive_publisher");
    ros::NodeHandle n;

    // --- Create Publishers ---
    // Only create publishers for the right controller's pose and trigger
    ros::Publisher pub_controller_pose = n.advertise<geometry_msgs::PoseStamped>("/vive_rightPose", 1);
    ros::Publisher pub_controller_trigger = n.advertise<std_msgs::Float32>("/vive_rightTrigger", 1);

    // Create the controller handler object
    VR_ControlHandler controlHandler;
    
    ROS_INFO("Started vive_pose node for RIGHT controller...");

    ros::Rate loop_rate(10); // 10 Hz loop

    // --- Main Loop ---
    while (ros::ok()) {
        // Get the latest controller state
        GetControllerState(controlHandler);

        // Check if the right controller's pose is valid
        if(controlHandler.pController[VRC_RIGHT]->status)
        {
            // Publish the pose
            pub_controller_pose.publish(controlHandler.pController[VRC_RIGHT]->pose.msg);
            
            // Publish the trigger value
            std_msgs::Float32 trigger_msg;
            trigger_msg.data = controlHandler.pController[VRC_RIGHT]->buttons.trigger;
            pub_controller_trigger.publish(trigger_msg);
        }

        // Update sequence
        ros::spinOnce();
        loop_rate.sleep();
    }

    // Shutdown OpenVR before exiting
    Shutdown();
    return 0;
}