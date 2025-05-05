#include <common_lib.h>

dummy_controller::dummy_controller()
    : LifecycleNode("dummy_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) 
{
    serial_port = this->declare_parameter("port", "/dev/ttyUSB0");
    baudrate = this->declare_parameter("baudrate", 115200);
    timeout_ms = this->declare_parameter("timeout", 1000);
    serial_timeout_sec = this->declare_parameter("serial_timeout_sec", 15);
    RCLCPP_INFO(this->get_logger(), "Serial port: %s", serial_port.c_str());
    RCLCPP_INFO(this->get_logger(), "Baudrate: %d", baudrate);
    RCLCPP_INFO(this->get_logger(), "Dummy Controller Initialized");
    RCLCPP_INFO(this->get_logger(), "Lifecycle Dummy controller node created.");
};

// Lifecycle state machine:
// [unconfigured] -- configure --> [inactive] -- activate --> [active]
//        ^                                   |
//        |                                   |
//     cleanup <-------------------------- deactivate
//        |
//     shutdown (any state can go here)

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
dummy_controller::on_configure(const rclcpp_lifecycle::State &) {
    init_dummy();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
dummy_controller::on_activate(const rclcpp_lifecycle::State &) {
    init_publisher_subscribe();
    RCLCPP_INFO(this->get_logger(), "Dummy Controller Activated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
dummy_controller::on_deactivate(const rclcpp_lifecycle::State &) {
    stop_dummy();
    home_dummy();
    reset_publisher_subscribe();
    RCLCPP_INFO(this->get_logger(), "Dummy Controller Deactivated");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
dummy_controller::on_cleanup(const rclcpp_lifecycle::State &) {
    stop_dummy();
    reset_dummy();
    reset_publisher_subscribe();
    turn_off_dummy();
    close_serial();
    RCLCPP_INFO(this->get_logger(), "Dummy Controller Cleaned, Serial port closed");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
dummy_controller::on_shutdown(const rclcpp_lifecycle::State &) {
    // 关闭手臂
    stop_dummy();
    reset_dummy();
    reset_publisher_subscribe();
    turn_off_dummy();
    close_serial();
    RCLCPP_INFO(this->get_logger(), "Dummy Controller Shutdown");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

dummy_controller::~dummy_controller() {
    // 关闭手臂
    stop_dummy();
    reset_dummy();
    reset_publisher_subscribe();
    turn_off_dummy();
    close_serial();
    RCLCPP_INFO(this->get_logger(), "Dummy controller destroyed");
};

void dummy_controller::init_dummy() {
    open_serial();
    start_dummy();
    home_dummy();
    RCLCPP_INFO(this->get_logger(), "Dummy Controller Initialized");
}

void dummy_controller::init_publisher_subscribe() {
    delta_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/dummy_arm/controller/pose", 10, 
        std::bind(&dummy_controller::end_pos_callback, this, std::placeholders::_1));
    
    joint_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/dummy/control/joint_trajectory_controller/command", 10,
        std::bind(&dummy_controller::joints_callback, this, std::placeholders::_1));
    
    j6_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dummy_arm/current/pose/j6", 10);    
    tool0_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dummy_arm/current/pose/tool0", 10);
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/dummy_arm/current/joint_states", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(25),
        std::bind(&dummy_controller::call_poll_position, this));
}

void dummy_controller::reset_publisher_subscribe() {
    delta_pose_sub_.reset();
    
    joint_sub_.reset();
    
    j6_pose_pub_.reset();
    tool0_pose_pub_.reset();
    joint_pub_.reset();
    
    tf_broadcaster_.reset();
    timer_.reset();
}

void dummy_controller::end_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    // 1. 提取位置
    double x = msg->pose.position.x;
    double y = msg->pose.position.y;
    double z = msg->pose.position.z;

    // 2. 四元數轉歐拉角 (rpy)
    tf2::Quaternion q(
        msg->pose.orientation.x,
        msg->pose.orientation.y,
        msg->pose.orientation.z,
        msg->pose.orientation.w
    );

    tf2::Matrix3x3 mat(q);
    double roll, pitch, yaw;
    mat.getRPY(roll, pitch, yaw);

    // 3. 轉成角度（degree）
    double rolld = roll * 180.0 / M_PI;
    double pitchd = pitch * 180.0 / M_PI;
    double yawd = yaw * 180.0 / M_PI;

    // 5. 發送串口
    pause_lpos = true;
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);  // 保留兩位小數
    oss << '@' << x << ',' << y << ',' << z << ','
        << rolld << ',' << pitchd << ',' << yawd;

    std::string cmd_body = oss.str();
    send_serial_command(cmd_body);
    // RCLCPP_INFO(this->get_logger(), "Sent command: %s", cmd_body.c_str());
    pause_lpos = false;
}

void dummy_controller::call_poll_position() {
    if (pause_lpos){
        return;
    }
    if (!serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }

    //读取并发布关节角度
    send_serial_command("#GETJPOS");

    //读取并发布末端位置
    send_serial_command("#GETLPOS");
}

void dummy_controller::async_receive_message() {
    if (!serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    auto port = serial_driver_->port();
    port->async_receive([this](const std::vector<uint8_t>& data, const size_t& size) {
        if (size > 0) {
            std::string msg(data.begin(), data.begin() + size);
            // RCLCPP_INFO(this->get_logger(), "Received: %s", msg.c_str());
            handle_received_message(msg);
        }
    
        if (serial_driver_->port()->is_open()) {
            async_receive_message();
        }
    });
}

void dummy_controller::handle_received_message(const std::string & msg) {
    // 拿掉 ID=xxx 後的訊息主體
    std::string body = msg;
    if (body.size() > 5 && body.rfind("ok") == body.size() - 2) {
        pause_cmd = false;
        RCLCPP_INFO(this->get_logger(), "Status: %s", body.c_str());
    } else if (body.find("ok JPOS") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "Get JPOS %s", body.c_str());
        std::vector<float> jpos = extract_floats(body);
        if (jpos.size() == 6)
            handle_joint_position(jpos);
        else
            RCLCPP_WARN(this->get_logger(), "Invalid JPOS value count");
    } else if (body.find("ok LPOS") != std::string::npos) {
        RCLCPP_INFO(this->get_logger(), "Get LPOS %s", body.c_str());
        std::vector<float> lpos = extract_floats(body);
        if (lpos.size() == 6)
            handle_linear_position(lpos);
        else
            RCLCPP_WARN(this->get_logger(), "Invalid LPOS value count");
    } else {
        RCLCPP_WARN(this->get_logger(), "Unrecognized message: %s", msg.c_str());
    }
}

std::vector<float> dummy_controller::extract_floats(const std::string& str) {
    std::stringstream ss(str);
    std::string token;
    std::vector<float> result;

    while (ss >> token) {
        try {
            float val = std::stof(token);
            result.push_back(val);
        } catch (...) {
            // 跳過非數值字串
        }
    }

    return result;
}

void dummy_controller::handle_joint_position(const std::vector<float>& jpos) {
    try {
        sensor_msgs::msg::JointState joint_state;
        // rclcpp::Time stamp(static_cast<uint64_t>(millis)/1000);  // 毫秒 → 奈秒
        rclcpp::Time stamp = this->now();
        joint_state.header.stamp = stamp;
        joint_state.name = joint_names_;
        for (size_t i = 0; i < jpos.size(); ++i) {
            joint_state.position.push_back(jpos[i]);
        }
        joint_pub_->publish(joint_state);

        RCLCPP_INFO(this->get_logger(), "Publish JPOS: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f",
            joint_state.position[0], joint_state.position[1], joint_state.position[2],
            joint_state.position[3], joint_state.position[4], joint_state.position[5]);
    }catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Parse joint error: %s", e.what());
    }
}

void dummy_controller::handle_linear_position(const std::vector<float>& lpos) {
    try {
        double x = lpos[0] / 1e3;
        double y = lpos[1] / 1e3;
        double z = lpos[2] / 1e3;
        double roll = lpos[3];
        double pitch = lpos[4];
        double yaw = lpos[5];

        RCLCPP_INFO(this->get_logger(), "Publish LPOS: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f", x, y, z, roll, pitch, yaw);
        // Convert roll, pitch, yaw to quaternion
        double rolld = roll * M_PI / 180.0;
        double pitchd = pitch * M_PI / 180.0;
        double yawd = yaw * M_PI / 180.0;
        tf2::Quaternion q;
        q.setRPY(rolld, pitchd, yawd);

        geometry_msgs::msg::PoseStamped j6_pose;
        rclcpp::Time stamp = this->now();
        j6_pose.header.stamp = stamp;
        j6_pose.header.frame_id = "base_link";
        j6_pose.pose.position.x = x;
        j6_pose.pose.position.y = y;
        j6_pose.pose.position.z = z;
        j6_pose.pose.orientation.x = q.x();
        j6_pose.pose.orientation.y = q.y();
        j6_pose.pose.orientation.z = q.z();
        j6_pose.pose.orientation.w = q.w();
        current_pose_ = {x, y, z, q.x(), q.y(), q.z(), q.w()};

        j6_pose_pub_->publish(j6_pose);

        geometry_msgs::msg::TransformStamped j6_tf_msg;
        j6_tf_msg.header.stamp = stamp;
        j6_tf_msg.header.frame_id = "base_link";
        j6_tf_msg.child_frame_id = "joint6";
        j6_tf_msg.transform.translation.x = x;
        j6_tf_msg.transform.translation.y = y;
        j6_tf_msg.transform.translation.z = z;
        j6_tf_msg.transform.rotation.x = q.x();
        j6_tf_msg.transform.rotation.y = q.y();
        j6_tf_msg.transform.rotation.z = q.z();
        j6_tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(j6_tf_msg);

        // 发布末端坐标
        // Step 2: 构造 J6 的 Transform
        tf2::Transform T_j6(q, tf2::Vector3(x, y, z));

        // Step 3: 构造 Z+0.07m 的偏移变换（在 J6 坐标系下）
        tf2::Transform T_offset;
        T_offset.setOrigin(tf2::Vector3(0.0, 0.0, 0.07));  // 7cm 前方
        T_offset.setRotation(tf2::Quaternion::getIdentity());  // 无旋转

        // Step 4: 得到 tool0 在 base_link 下的位姿
        tf2::Transform T_tool0 = T_j6 * T_offset;
        tf2::Quaternion q_tool0 = T_tool0.getRotation();
        tf2::Vector3 p_tool0 = T_tool0.getOrigin();

        // Step 5: 发布 PoseStamped
        geometry_msgs::msg::PoseStamped tool0_pose;
        tool0_pose.header.stamp = stamp;
        tool0_pose.header.frame_id = "base_link";

        tool0_pose.pose.position.x = p_tool0.x();
        tool0_pose.pose.position.y = p_tool0.y();
        tool0_pose.pose.position.z = p_tool0.z();

        tool0_pose.pose.orientation.x = q_tool0.x();
        tool0_pose.pose.orientation.y = q_tool0.y();
        tool0_pose.pose.orientation.z = q_tool0.z();
        tool0_pose.pose.orientation.w = q_tool0.w();

        // 发布
        tool0_pose_pub_->publish(tool0_pose);


        geometry_msgs::msg::TransformStamped tool0_tf_msg;
        tool0_tf_msg.header.stamp = stamp;
        tool0_tf_msg.header.frame_id = "joint6";         // 父坐标系
        tool0_tf_msg.child_frame_id = "tool0";           // 子坐标系

        // 平移：沿 Z 正方向 7cm
        tool0_tf_msg.transform.translation.x = 0.0;
        tool0_tf_msg.transform.translation.y = 0.0;
        tool0_tf_msg.transform.translation.z = 0.07;

        // 旋转：无旋转
        tool0_tf_msg.transform.rotation.x = 0.0;
        tool0_tf_msg.transform.rotation.y = 0.0;
        tool0_tf_msg.transform.rotation.z = 0.0;
        tool0_tf_msg.transform.rotation.w = 1.0;

        // 发布
        tf_broadcaster_->sendTransform(tool0_tf_msg);

        geometry_msgs::msg::TransformStamped camera_tf_msg;
        camera_tf_msg.header.stamp = stamp;
        camera_tf_msg.header.frame_id = "joint6";         // 父坐标系
        camera_tf_msg.child_frame_id = "camera_frame";           // 子坐标系

        // 平移：沿 Z 正方向 2.5cm
        camera_tf_msg.transform.translation.x = -0.04;
        camera_tf_msg.transform.translation.y = 0.02;
        camera_tf_msg.transform.translation.z = 0.025;

        // 旋转：无旋转
        camera_tf_msg.transform.rotation.x = 0.0;
        camera_tf_msg.transform.rotation.y = 0.0;
        camera_tf_msg.transform.rotation.z = 0.0;
        camera_tf_msg.transform.rotation.w = 1.0;

        // 发布
        tf_broadcaster_->sendTransform(camera_tf_msg);

    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Parse pose error: %s", e.what());
    }
}

void dummy_controller::joints_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
    if (msg->points.empty()) {
        RCLCPP_WARN(this->get_logger(), "Received empty trajectory.");
        return;
    }

    const auto &positions = msg->points[0].positions;
    
    if (positions.size() < 6) {
        RCLCPP_WARN(this->get_logger(), "Expected 6 joint values, got %ld", positions.size());
        return;
    }
    //读取并发布关节角度
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3);
    oss << "&" << positions[0] << "," << positions[1] << "," << positions[2] << ","
        << positions[3] << "," << positions[4] << "," << positions[5];
    std::string cmd_body = oss.str();
    send_serial_command(cmd_body);
    // RCLCPP_INFO(this->get_logger(), "Sent command: %s", cmd_body.c_str());
}

void dummy_controller::open_serial() {
    // 打開串口
    drivers::serial_driver::SerialPortConfig config(
        baudrate,                       // baud_rate
        drivers::serial_driver::FlowControl::NONE,
        drivers::serial_driver::Parity::NONE,
        drivers::serial_driver::StopBits::ONE
    );
    try {
        io_context_ = std::make_shared<drivers::common::IoContext>(1);
        serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
        serial_driver_ = std::make_shared<drivers::serial_driver::SerialDriver>(*io_context_);
        serial_driver_->init_port(serial_port, config);
        serial_driver_->port()->open();
        async_receive_message();
        if (!serial_driver_->port()->is_open()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        return;
    }
}

void dummy_controller::start_dummy() {
    // 打開手臂電機
    if (!serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    send_highlevel_serial_command("!START");
}

void dummy_controller::reset_dummy() {
    pause_lpos = true;
    // 回到收起位置
    if (!serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    send_highlevel_serial_command("!RESET");
    pause_lpos = false;
}

void dummy_controller::home_dummy() {
    pause_lpos = true;
    // 回到7字位置
    if (!serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    send_highlevel_serial_command("!HOME");
    pause_lpos = false;
}

void dummy_controller::stop_dummy() {
    pause_lpos = true;
    // 停止手臂
    if (!serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    send_highlevel_serial_command("!STOP");
    pause_lpos = false;
}

void dummy_controller::turn_off_dummy() {
    // 关闭手臂电机
    if (!serial_driver_->port()->is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    send_highlevel_serial_command("!DISABLE");
}

void dummy_controller::close_serial() {
    // 关闭串口
    if (serial_driver_->port()->is_open()) {
        serial_driver_->port()->close();
    }
    RCLCPP_INFO(this->get_logger(), "Serial port closed");
}

void dummy_controller::send_highlevel_serial_command(const std::string &cmd) {
    try {
        //读取并发布关节角度
        // std::string id = generate_id();
        std::ostringstream oss;
        oss << cmd << "\n";
        std::string cmd_str = oss.str();
        // RCLCPP_INFO(this->get_logger(), "Send heighlevel command: %s", cmd_str.c_str());
        serial_driver_->port()->send(std::vector<uint8_t>(cmd_str.begin(), cmd_str.end()));
        // serial_driver_->port()->send(std::vector<uint8_t>(cmd_str.begin(), cmd_str.end()));
        RCLCPP_INFO(this->get_logger(), "Send command: %s", cmd_str.c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
    }
}

void dummy_controller::send_serial_command(const std::string &cmd) {
    try {
        //读取并发布关节角度
        // std::string id = generate_id();
        std::ostringstream oss;
        oss << cmd << "\n";
        std::string cmd_str = oss.str();
        // serial_driver_->port()->async_send(std::vector<uint8_t>(cmd_str.begin(), cmd_str.end()));
        serial_driver_->port()->send(std::vector<uint8_t>(cmd_str.begin(), cmd_str.end()));
        RCLCPP_INFO(this->get_logger(), "Send command: %s", cmd_str.c_str());
        // serial_driver_->port()->send(std::vector<uint8_t>(cmd_str.begin(), cmd_str.end()));
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
    }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dummy_controller>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}