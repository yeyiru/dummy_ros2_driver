#include <common_lib.h>

dummy_controller::dummy_controller()
    : LifecycleNode("dummy_controller"),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_) 
{
    serial_port = this->declare_parameter("port", "/dev/ttyUSB0");
    baudrate = this->declare_parameter("baudrate", 115200);
    timeout_ms = this->declare_parameter("timeout", 1000);
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
    turn_off_dummy();
    reset_publisher_subscribe();
    close_serial();
    RCLCPP_INFO(this->get_logger(), "Dummy controller destroyed");
};

void dummy_controller::init_dummy() {
    try {
        serial_.setPort(serial_port);
        serial_.setBaudrate(baudrate);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(timeout_ms);
        serial_.setTimeout(timeout);
        serial_.open();
        if (!serial_.isOpen()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port");
            return;
        }
        RCLCPP_INFO(this->get_logger(), "Starting ...");
        while (rclcpp::ok()) {
            serial_.flushInput();
            serial_.write("!START\n");
            std::this_thread::sleep_for(std::chrono::seconds(1));

            std::string data = serial_.readline();
            RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());

            if (data.find("Started ok") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "Dummy started");
                serial_.flushInput();
                serial_.write("!HOME\n");
                break;
            }
        } 
    } catch (const std::exception & e) {
        RCLCPP_ERROR(this->get_logger(), "Error: %s", e.what());
        return;
    }
}

void dummy_controller::init_publisher_subscribe() {
    delta_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/dummy_arm/controller/pose", 10, 
        std::bind(&dummy_controller::end_pos_callback, this, std::placeholders::_1));
    
    joint_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
        "/dummy/control/joint_trajectory_controller/command", 10,
        std::bind(&dummy_controller::joints_callback, this, std::placeholders::_1));
    
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/dummy_arm/current/pose", 10);    
    joint_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/dummy_arm/current/joint_states", 10);
    
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&dummy_controller::poll_position, this));
}

void dummy_controller::reset_publisher_subscribe() {
    delta_pose_sub_.reset();
    
    joint_sub_.reset();
    
    pose_pub_.reset();    
    joint_pub_.reset();
    
    tf_broadcaster_.reset();
    timer_.reset();
}

void dummy_controller::end_pos_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
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

    // 4. 組成字串
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);  // 保留兩位小數
    oss << '@' << x << ',' << y << ',' << z << ','
        << rolld << ',' << pitchd << ',' << yawd;

    std::string cmd_str = oss.str();

    // 5. 發送串口
    pause_lpos = true;
    serial_.flushInput();
    serial_.write(cmd_str);  // 假設你已有 serial_.write() 方法
    RCLCPP_INFO(this->get_logger(), "Sent command: %s", cmd_str.c_str());
    pause_lpos = false;
}

void dummy_controller::poll_position() {
    if (pause_lpos){
        return;
    }
    if (serial_.isOpen()) {
        //读取并发布关节角度
        serial_.flushInput();
        serial_.write("#GETJPOS\n");
        std::string j_response = serial_.readline(100, "\n");
        j_response = read_response(j_response);
        if (j_response == "OUT") {
            return;  // 無效回應，跳過處理
        }

        std::vector<std::string> j_tokens;
        std::stringstream j_ss(j_response.substr(0));
        std::string j_item;
        while (std::getline(j_ss, j_item, ' ')) {
            if (!j_item.empty()) {  // 避免多個空格導致空 token
                j_tokens.push_back(j_item);
            }
        }

        if (j_tokens.size() != 7 || j_tokens[0] != "ok") {
            RCLCPP_WARN(this->get_logger(), "Malformed joints response: '%s'", j_response.c_str());
            return;
        }
        try {
            sensor_msgs::msg::JointState joint_state;
            joint_state.header.stamp = this->now();
            joint_state.name = joint_names_;
            for (size_t i = 1; i < j_tokens.size(); ++i) {
                double angle = std::stod(j_tokens[i]);
                joint_state.position.push_back(angle);
            }
            joint_pub_->publish(joint_state);

            RCLCPP_INFO(this->get_logger(), "Publish joint angles: j1: %f, j2: %f, j3: %f, j4: %f, j5: %f, j6: %f",
                joint_state.position[0], joint_state.position[1], joint_state.position[2],
                joint_state.position[3], joint_state.position[4], joint_state.position[5]);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Parse joint error: %s", e.what());
        }

        // 读取并发布末端位置坐标
        serial_.flushInput();
        serial_.write("#GETLPOS\n");
        std::string p_response = serial_.readline(100, "\n");
        p_response = read_response(p_response);
        if (p_response == "OUT") {
            return;  // 無效回應，跳過處理
        }

        std::vector<std::string> p_tokens;
        std::stringstream p_ss(p_response.substr(0));
        std::string p_item;
        while (std::getline(p_ss, p_item, ' ')) {
            if (!p_item.empty()) {  // 避免多個空格導致空 token
                p_tokens.push_back(p_item);
            }
        }

        if (p_tokens.size() != 7 || p_tokens[0] != "ok") {
            RCLCPP_WARN(this->get_logger(), "Malformed pose response: '%s'", p_response.c_str());
            return;
        }

        try {
            double x = std::stod(p_tokens[1]);
            double y = std::stod(p_tokens[2]);
            double z = std::stod(p_tokens[3]);
            double roll = std::stod(p_tokens[4]);
            double pitch = std::stod(p_tokens[5]);
            double yaw = std::stod(p_tokens[6]);

            RCLCPP_INFO(this->get_logger(), "Publish pose: x: %f, y: %f, z: %f, roll: %f, pitch: %f, yaw: %f", x, y, z, roll, pitch, yaw);

            tf2::Quaternion q;
            q.setRPY(roll, pitch, yaw);

            geometry_msgs::msg::PoseStamped pose;
            pose.header.stamp = this->now();
            pose.header.frame_id = "base_link";
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = z;
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();
            current_pose_ = {x, y, z, q.x(), q.y(), q.z(), q.w()};

            pose_pub_->publish(pose);

            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = this->now();
            tf_msg.header.frame_id = "base_link";
            tf_msg.child_frame_id = "tool0";
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = z;
            tf_msg.transform.rotation.x = q.x();
            tf_msg.transform.rotation.y = q.y();
            tf_msg.transform.rotation.z = q.z();
            tf_msg.transform.rotation.w = q.w();

            tf_broadcaster_->sendTransform(tf_msg);

        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Parse pose error: %s", e.what());
        }
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

    std::ostringstream cmd;
    cmd << std::fixed << std::setprecision(3);
    cmd << "&" << positions[0] << "," << positions[1] << "," << positions[2] << ","
        << positions[3] << "," << positions[4] << "," << positions[5] << "\n";

    try {
        serial_.flushInput();
        serial_.write(cmd.str());
        RCLCPP_INFO(this->get_logger(), "Sent: %s", cmd.str().c_str());
    } catch (const std::exception &e) {
        RCLCPP_ERROR(this->get_logger(), "Serial write error: %s", e.what());
    }
}

void dummy_controller::reset_dummy() {
    pause_lpos = true;
    // 回到收起位置
    if (!serial_.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    serial_.flushInput();
    serial_.write("!RESET\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::string data = serial_.readline();
    RCLCPP_INFO(this->get_logger(), "Dummy Reset(%s)", data.c_str());
    pause_lpos = false;
}

void dummy_controller::home_dummy() {
    pause_lpos = true;
    // 回到7字位置
    if (!serial_.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    serial_.flushInput();
    serial_.write("!HOME\n");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::string data = serial_.readline();
    // RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());
    RCLCPP_INFO(this->get_logger(), "Dummy Homed(%s)", data.c_str());
    pause_lpos = false;
}

void dummy_controller::stop_dummy() {
    pause_lpos = true;
    // 停止手臂
    if (!serial_.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    serial_.flushInput();
    serial_.write("!STOP\n");
    std::string data = serial_.readline();
    // RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());
    RCLCPP_INFO(this->get_logger(), "Dummy Stopped(%s)", data.c_str());
    pause_lpos = false;
}

void dummy_controller::turn_off_dummy() {
    // 关闭手臂电机
    if (!serial_.isOpen()) {
        RCLCPP_ERROR(this->get_logger(), "Serial port is not open");
        return;
    }
    serial_.flushInput();
    serial_.write("!DISABLE\n");
    std::string data = serial_.readline();
    // RCLCPP_INFO(this->get_logger(), "Received: %s", data.c_str());
    RCLCPP_INFO(this->get_logger(), "Dummy Disabled(%s)", data.c_str());
}

void dummy_controller::close_serial() {
    // 关闭串口
    if (serial_.isOpen()) {
        serial_.close();
        RCLCPP_INFO(this->get_logger(), "Serial port closed");
    } else {
        RCLCPP_WARN(this->get_logger(), "Serial port is not open");
    }
}

std::string dummy_controller::read_response(const std::string &response_raw) {
    std::string response = response_raw;
    // 去除尾部的 \r 或 \n
    response.erase(std::remove(response.begin(), response.end(), '\r'), response.end());
    response.erase(std::remove(response.begin(), response.end(), '\n'), response.end());

    // 檢查是否過短
    if (response.size() < 5) {
        RCLCPP_WARN(this->get_logger(), "Response too short, skipping: '%s'", response.c_str());
        return "OUT";
    }

    // 檢查是否是 "15ok" 類型的確認消息
    if (response.find("15ok") != std::string::npos) {
        // RCLCPP_INFO(this->get_logger(), "Skip response (ack): '%s'", response.c_str());
        return "OUT";
    }
    return response;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<dummy_controller>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}