#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from cv_bridge import CvBridge
import cv2
import numpy as np

class DummyUSBCameraPublisher(Node):
    def __init__(self):
        super().__init__('dummy_usb_cam_publisher')

        # 读取参数
        self.declare_parameters(namespace='',
            parameters=[
                ('camera_intrinsics.fx', 0.0),
                ('camera_intrinsics.fy', 0.0),
                ('camera_intrinsics.cx', 0.0),
                ('camera_intrinsics.cy', 0.0),
                ('distortion.d1', 0.0),
                ('distortion.d2', 0.0),
                ('distortion.d3', 0.0),
                ('distortion.d4', 0.0),
                ('distortion.d5', 0.0),
                ('image_width', 640),
                ('image_height', 480),
                ('fps', 30),
                ('camera_frame_id', 'camera'),
                ('camera_topic', '/camera/image_raw'),
            ]
        )

        self.bridge = CvBridge()

        # 打开相机
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.get_logger().error("Cannot open USB camera")
            exit(1)

        # 图像发布器
        topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.pub_image = self.create_publisher(Image, topic, 10)
        self.pub_info = self.create_publisher(CameraInfo, topic + '/camera_info', 10)
        # self.pub_compressed = self.create_publisher(CompressedImage, topic + '/compressed', 10)

        # 构造 CameraInfo
        self.camera_info = self.build_camera_info()

        # 定时器
        timer_period = 1.0 / self.get_parameter('fps').value
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def build_camera_info(self):
        info = CameraInfo()
        self.width = self.get_parameter('image_width').value
        self.height = self.get_parameter('image_height').value
        fx = self.get_parameter('camera_intrinsics.fx').value
        fy = self.get_parameter('camera_intrinsics.fy').value
        cx = self.get_parameter('camera_intrinsics.cx').value
        cy = self.get_parameter('camera_intrinsics.cy').value
        d1 = self.get_parameter('distortion.d1').value
        d2 = self.get_parameter('distortion.d2').value
        d3 = self.get_parameter('distortion.d3').value
        d4 = self.get_parameter('distortion.d4').value
        d5 = self.get_parameter('distortion.d5').value

        info.width = self.width
        info.height = self.height
        info.distortion_model = 'plumb_bob'
        info.d = [d1, d2, d3, d4, d5]
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0,
                  0.0, fy, cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        return info

    def timer_callback(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Failed to grab frame")
            return

        frame = cv2.resize(frame, (
            self.get_parameter('image_width').value,
            self.get_parameter('image_height').value
        ))
        center_x = self.width // 2
        center_y = self.height // 2

        # 画水平线
        cv2.line(frame, (center_x - 100, center_y), (center_x + 100, center_y), (0, 255, 0), 2)

        # 画垂直线
        cv2.line(frame, (center_x, center_y - 100), (center_x, center_y + 100), (0, 255, 0), 2)
        
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.get_parameter('camera_frame_id').value

        self.camera_info.header = msg.header

        self.pub_image.publish(msg)
        self.pub_info.publish(self.camera_info)

    

    def destroy_node(self):
        self.cap.release()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = DummyUSBCameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()