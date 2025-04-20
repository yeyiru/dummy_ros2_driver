#ifndef DUMMY_H
#define DUMMY_H

#include <stdio.h>
#include <string>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <serial/serial.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class dummy_controller : public rclcpp::Node
{
public:
    dummy_controller();
    ~dummy_controller();
    void init_dummy();
    void init_publisher_subscribe();

    void poll_position();

    std::string read_response(const std::string &response_raw);
    
    void set_pid(const std::vector<double> & pid);
    
    
    void control_end_position(const std::vector<double> & position);

    void joints_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void end_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    serial::Serial serial_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr delta_pose_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string serial_port;
    int baudrate;
    int timeout_ms;

    std::vector<std::string> joint_names_ = {
    "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    std::vector<double> current_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

#endif