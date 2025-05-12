#ifndef USBCAM_H
#define USBCAM_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

class usb_cam_publisher : public rclcpp::Node {
public:
    usb_cam_publisher();
    ~usb_cam_publisher();
    void publish_frame();

    cv::VideoCapture cap_;
    image_transport::Publisher publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    sensor_msgs::msg::CameraInfo camera_info_msg_;

    double fx, fy, cx, cy;
    double d1, d2, d3, d4, d5;
    int width, height, fps;

    std::string camera_frame_id, camera_topic;
};

#endif