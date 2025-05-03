#ifndef DUMMY_H
#define DUMMY_H

#include <stdio.h>
#include <string>
#include <optional>
#include <rclcpp/rclcpp.hpp>
// #include <serial/serial.h>
// #include <boost/asio.hpp>
// #include <boost/bind/bind.hpp>
#include <serial_driver/serial_driver.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

class dummy_controller : public rclcpp_lifecycle::LifecycleNode {
public:
    dummy_controller();
    ~dummy_controller();
    
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);
    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);
    
    void init_dummy();
    void init_publisher_subscribe();
    void reset_publisher_subscribe();

    // void poll_position();
    void call_poll_position();
    void async_receive_message();
    void handle_received_message(const std::string &msg);
    std::vector<float> extract_floats(const std::string &str);
    void handle_joint_position(const std::vector<float>& jpos, const int64_t& id);
    void handle_linear_position(const std::vector<float>& lpos, const int64_t& id);


    // std::string read_response(const std::string &response_raw);
    std::string send_serial_command(const std::string &cmd);

    void joints_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg);
    void end_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

    void open_serial();
    void start_dummy();
    void reset_dummy();
    void home_dummy();
    void stop_dummy();
    void turn_off_dummy();
    void close_serial();

    std::string generate_id();
    // void handle_response(const std::vector<uint8_t> & data);

    bool waitting_for_manipulate(const std::string &data, const std::chrono::steady_clock::time_point &start_timestamp);
    // void control_end_position(const std::vector<double> & position);

    // serial::Serial serial_;
    // drivers::serial_driver::SerialPortConfig serial_config_;
    std::shared_ptr<drivers::serial_driver::SerialDriver> serial_driver_;
    std::shared_ptr<drivers::common::IoContext> io_context_;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr delta_pose_sub_;
    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr joint_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr j6_pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr tool0_pose_pub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    std::string serial_port;

    std::unordered_map<std::string, std::string> pending_requests_;

    std::vector<uint8_t> transmit_data_buffer = std::vector<uint8_t>(1024); // 发送缓冲区
    std::vector<uint8_t> receive_data_buffer = std::vector<uint8_t>(1024);  // 接收缓冲区


    int seq_counter_;
    std::unordered_map<std::string, std::string> pending_commands_;  // ID -> CMD

    int baudrate;
    int timeout_ms;
    int serial_timeout_sec;

    bool pause_lpos = false;

    std::vector<std::string> joint_names_ = {
        "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};
    std::vector<double> current_pose_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

#endif