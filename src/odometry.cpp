//#######################################################################################################
// Name:             odometry.cpp
// Purpose:          To calculate the robot odometry for use in SLAM and NAVIGATION
// Description:      to be used with esp32 firmware and micro-ros
// Related Files:         
// Author:           logan naidoo, south africa, 2024
//########################################################################################################
#include "odometry.h"
#include <math.h>
#include <nav_msgs/msg/odometry.h>
#include <rosidl_runtime_c/string_functions.h>  // Include this for string functions
#include <esp_timer.h>  // Include for esp_timer_get_time()


// builtin_interfaces__msg__Time current_ros_time;
// // clock callback to sync host pc Ros clock with ESP32 needed to populate Odometry Header
// void clock_callback(const void * msgin) {
//     const builtin_interfaces__msg__Time * msg = (const builtin_interfaces__msg__Time *)msgin;
//     current_ros_time.sec = msg->sec;
//     current_ros_time.nanosec = msg->nanosec;
// }

// Helper function to convert yaw (theta) to quaternion
void yawToQuaternion(float yaw, geometry_msgs__msg__Quaternion &q) {
    q.x = 0.0;
    q.y = 0.0;
    q.z = sin(yaw / 2.0);
    q.w = cos(yaw / 2.0);
}

// Constructor
Odometry::Odometry(float wheel_radius, float wheel_base_width)
    : x(0), y(0), theta(0), kinematics(wheel_radius, wheel_base_width) {
    // Initialize the odometry message
    rosidl_runtime_c__String__init(&odometry_msg.header.frame_id);
    rosidl_runtime_c__String__init(&odometry_msg.child_frame_id);
    rosidl_runtime_c__String__assign(&odometry_msg.header.frame_id, "odom");
    rosidl_runtime_c__String__assign(&odometry_msg.child_frame_id, "base_footprint");  // was base_link
}

// Update function that calculates odometry and populates the message
void Odometry::update(float front_left_velocity, float front_right_velocity, 
                        float back_left_velocity, float back_right_velocity, 
                        float dt, builtin_interfaces__msg__Time current_ros_time) {
    float linear_velocity, angular_velocity;
    WheelSpeeds speeds = {front_left_velocity, front_right_velocity, back_left_velocity, back_right_velocity};
    kinematics.forwardKinematics(speeds, linear_velocity, angular_velocity);

    // Update the robot's position and orientation
    x += linear_velocity * cos(theta) * dt;
    y += linear_velocity * sin(theta) * dt;
    theta += angular_velocity * dt;

    // Populate the odometry message with synchronized ROS 2 time
    odometry_msg.header.stamp.sec = current_ros_time.sec;
    odometry_msg.header.stamp.nanosec = current_ros_time.nanosec;

    // Position
    odometry_msg.pose.pose.position.x = x;
    odometry_msg.pose.pose.position.y = y;
    odometry_msg.pose.pose.position.z = 0.0;

    // Orientation (quaternion)
    yawToQuaternion(theta, odometry_msg.pose.pose.orientation);

    // Velocity (twist)
    odometry_msg.twist.twist.linear.x = linear_velocity;
    odometry_msg.twist.twist.angular.z = angular_velocity;
}

// Function to return the robot's pose (unchanged)
void Odometry::getPose(float &x_out, float &y_out, float &theta_out) {
    x_out = x;
    y_out = y;
    theta_out = theta;
}

// Function to return the odometry message (for publishing in main.cpp)
nav_msgs__msg__Odometry Odometry::getOdometryMsg() const {
    return odometry_msg;
}

