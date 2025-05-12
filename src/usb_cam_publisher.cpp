#include <usb_cam_publisher.hpp>

usb_cam_publisher::usb_cam_publisher()
    : Node("dummy_usb_cam_publisher")
{
    // Read some camera parameters from the yaml file
    fx = this->declare_parameter("camera_intrinsics.fx", 0.0);
    fy = this->declare_parameter("camera_intrinsics.fy", 0.0);
    cx = this->declare_parameter("camera_intrinsics.cx", 0.0);
    cy = this->declare_parameter("camera_intrinsics.cy", 0.0);

    d1 = this->declare_parameter("distortion.d1", 0.0);
    d2 = this->declare_parameter("distortion.d2", 0.0);
    d3 = this->declare_parameter("distortion.d3", 0.0);
    d4 = this->declare_parameter("distortion.d4", 0.0);
    d5 = this->declare_parameter("distortion.d5", 0.0);

    width = this->declare_parameter("image_width", 640);
    height = this->declare_parameter("image_height", 480);

    fps = this->declare_parameter("fps", 10);

    camera_frame_id = this->declare_parameter("camera_frame_id", "camera_frame");
    camera_topic = this->declare_parameter("camera_topic", "image_raw");
    RCLCPP_INFO(this->get_logger(), "Camera frame id: %s", camera_frame_id.c_str());
    // Initialize the camera
    cap_.open(0, cv::CAP_V4L2);  // Open the default camera (0)
    if (!cap_.isOpened()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open camera");
        return;
    }

    // Initialize the image publisher
    publisher_ = image_transport::create_publisher(this, camera_topic);
    
    std::stringstream camera_info_topic;
    camera_info_topic << camera_topic << "/camera_info";
    camera_info_pub_ = this->create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic.str(), 10);
    // Create a timer to publish frames at a fixed rate
    int interval_ms = static_cast<int>(std::round(1000.0 / fps));
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(interval_ms),  // Publish every 100 ms
        std::bind(&usb_cam_publisher::publish_frame, this));
}


usb_cam_publisher::~usb_cam_publisher()
{
    // Release the camera
    if (cap_.isOpened()) {
        cap_.release();
    }
}

void usb_cam_publisher::publish_frame()
{
    cv::Mat frame;
    cap_ >> frame;  // Capture a new frame

    if (frame.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to capture frame");
        return;
    }

    // Convert the frame to a ROS image message
    sensor_msgs::msg::Image::SharedPtr msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();

    // Publish the image
    publisher_.publish(msg);

    // 設定 camera_info_msg_
    camera_info_msg_.width = width;
    camera_info_msg_.height = height;
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.d = {d1, d2, d3, d4, d5};

    camera_info_msg_.k = {fx, 0.0, cx,
                        0.0, fy, cy,
                        0.0, 0.0, 1.0};

    camera_info_msg_.r = {1.0, 0.0, 0.0,
                        0.0, 1.0, 0.0,
                        0.0, 0.0, 1.0};

    camera_info_msg_.p = {fx, 0.0, cx, 0.0,
                        0.0, fy, cy, 0.0,
                        0.0, 0.0, 1.0, 0.0};
    RCLCPP_INFO(this->get_logger(), "Publishing Camera Frame Success");
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<usb_cam_publisher>());
  rclcpp::shutdown();
  return 0;
}