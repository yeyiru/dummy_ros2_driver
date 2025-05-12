import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

class DebugPosePublisher(Node):
    def __init__(self):
        super().__init__('debug_pose_publisher')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.j6_pose_pub = self.create_publisher(PoseStamped, '/dummy_arm/current/pose/j6', 10)
        self.tool0_pose_pub = self.create_publisher(PoseStamped, '/dummy_arm/current/pose/tool0', 10)
        self.joint_pub = self.create_publisher(JointState, '/dummy_arm/current/joint_states', 10)

        self.timer = self.create_timer(0.05, self.timer_callback)  # 每秒发布一次
        self.get_logger().info("Debug TF publisher started.")

    def timer_callback(self):
        current_end_pose = [222.0, 0.0, 307.0, 0.0, 90.0, 0.0]
        self.handle_current_end_pose(current_end_pose)
        self.publish_joint_states()

    def publish_joint_states(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        msg.position = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]  # in degrees or radians depending on consumer
        self.joint_pub.publish(msg)
        self.get_logger().info('Published dummy joint states.')

    def handle_current_end_pose(self, current_end_pose):
        try:
            x = current_end_pose[0] / 1000.0
            y = current_end_pose[1] / 1000.0
            z = current_end_pose[2] / 1000.0
            roll = current_end_pose[3]
            pitch = current_end_pose[4]
            yaw = current_end_pose[5]

            rolld = np.radians(roll)
            pitchd = np.radians(pitch)
            yawd = np.radians(yaw)

            quat = R.from_euler('xyz', [rolld, pitchd, yawd]).as_quat()
            stamp = self.get_clock().now().to_msg()

            # J6 Pose
            j6_pose = PoseStamped()
            j6_pose.header.stamp = stamp
            j6_pose.header.frame_id = "base_link"
            j6_pose.pose.position.x = x
            j6_pose.pose.position.y = y
            j6_pose.pose.position.z = z
            j6_pose.pose.orientation.x = quat[0]
            j6_pose.pose.orientation.y = quat[1]
            j6_pose.pose.orientation.z = quat[2]
            j6_pose.pose.orientation.w = quat[3]
            self.j6_pose_pub.publish(j6_pose)

            # J6 TF
            j6_tf = TransformStamped()
            j6_tf.header.stamp = stamp
            j6_tf.header.frame_id = "base_link"
            j6_tf.child_frame_id = "joint6"
            j6_tf.transform.translation.x = x
            j6_tf.transform.translation.y = y
            j6_tf.transform.translation.z = z
            j6_tf.transform.rotation.x = quat[0]
            j6_tf.transform.rotation.y = quat[1]
            j6_tf.transform.rotation.z = quat[2]
            j6_tf.transform.rotation.w = quat[3]
            self.tf_broadcaster.sendTransform(j6_tf)

            # Tool0 Pose
            offset_z = 0.07
            tool0_pose = PoseStamped()
            tool0_pose.header.stamp = stamp
            tool0_pose.header.frame_id = "base_link"
            tool0_pose.pose.position.x = x
            tool0_pose.pose.position.y = y
            tool0_pose.pose.position.z = z + offset_z
            tool0_pose.pose.orientation.x = quat[0]
            tool0_pose.pose.orientation.y = quat[1]
            tool0_pose.pose.orientation.z = quat[2]
            tool0_pose.pose.orientation.w = quat[3]
            self.tool0_pose_pub.publish(tool0_pose)

            # Tool0 TF
            tool0_tf = TransformStamped()
            tool0_tf.header.stamp = stamp
            tool0_tf.header.frame_id = "joint6"
            tool0_tf.child_frame_id = "tool0"
            tool0_tf.transform.translation.x = 0.0
            tool0_tf.transform.translation.y = 0.0
            tool0_tf.transform.translation.z = 0.07
            tool0_tf.transform.rotation.x = 0.0
            tool0_tf.transform.rotation.y = 0.0
            tool0_tf.transform.rotation.z = 0.0
            tool0_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(tool0_tf)

            # Camera TF
            r = R.from_euler('z', -90, degrees=True)
            cam_quat = r.as_quat()
            camera_tf = TransformStamped()
            camera_tf.header.stamp = stamp
            camera_tf.header.frame_id = "joint6"
            camera_tf.child_frame_id = "camera_frame"
            camera_tf.transform.translation.x = -0.04
            camera_tf.transform.translation.y = 0.02
            camera_tf.transform.translation.z = 0.025
            camera_tf.transform.rotation.x = cam_quat[0]
            camera_tf.transform.rotation.y = cam_quat[1]
            camera_tf.transform.rotation.z = cam_quat[2]
            camera_tf.transform.rotation.w = cam_quat[3]
            self.tf_broadcaster.sendTransform(camera_tf)

            # ISAAC TF
            r = R.from_euler('z', 90, degrees=True)
            isaac_quat = r.as_quat()
            isaac_tf = TransformStamped()
            isaac_tf.header.stamp = stamp
            isaac_tf.header.frame_id = "base_link"
            isaac_tf.child_frame_id = "isaac_frame"
            isaac_tf.transform.translation.x = 0.0
            isaac_tf.transform.translation.y = 0.0
            isaac_tf.transform.translation.z = 0.0
            isaac_tf.transform.rotation.x = isaac_quat[0]
            isaac_tf.transform.rotation.y = isaac_quat[1]
            isaac_tf.transform.rotation.z = isaac_quat[2]
            isaac_tf.transform.rotation.w = isaac_quat[3]
            self.tf_broadcaster.sendTransform(isaac_tf)

            self.get_logger().info(
                f"Published LPOS: x={x:.3f}, y={y:.3f}, z={z:.3f}, roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}"
            )

        except Exception as e:
            self.get_logger().error(f"Parse pose error: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = DebugPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()