import rclpy

from rclpy.lifecycle import LifecycleNode
from rclpy.lifecycle import State
from rclpy.lifecycle import TransitionCallbackReturn
from rclpy.timer import Timer

from geometry_msgs.msg import PoseStamped, TransformStamped
from trajectory_msgs.msg import JointTrajectory
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformBroadcaster
from sensor_msgs.msg import JointState

import numpy as np
import dummy_cli_tool.ref_tool

class dummy_controller(LifecycleNode):
    def __init__(self):
        super().__init__('dummy_controller')
        self.dummy_driver = None
        
        # 7字位置
        # self.ready_joints = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        self.home_joints = np.array([0.0, 0.0, 90.0, 0.0, 0.0, 0.0])
        # Reset位置，收起
        self.reset_joints = np.array([0.0, -75.0, 180.0, 0.0, 0.0, 0.0])  
        
        self.joint_names = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"]
        self.get_logger().info("dummy_controller_node Lifecycle node constructor called")
    
    # ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ Lifecycle node callback ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    def on_configure(self, state: State) -> TransitionCallbackReturn:
        self.init_dummy()
        # 设为7字位置
        self.home_dummy()

        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(1)
        self.get_logger().info("on_configure() called")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        self.init_pub_sub()
        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(4)
        self.get_logger().info("on_activate() called")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        self.stop_dummy()
        # 设为7字位置
        self.home_dummy()

        self.reset_pub_sub()
        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(1)
        self.get_logger().info("on_deactivate() called")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        self.stop_dummy()
        self.reset_dummy()

        self.off_dummy()
        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(2)
        self.get_logger().info("on_cleanup() called")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: State) -> TransitionCallbackReturn:
        #0 RGB跑马灯  1白光快闪  2白光慢闪  3红光常亮   4绿光常亮   5蓝光常亮
        self.dummy_driver.robot.set_rgb_mode(0)
        self.get_logger().info("on_shutdown() called")
        return TransitionCallbackReturn.SUCCESS
    # ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ Lifecycle node callback ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
    
    def init_dummy(self):
        self.dummy_driver = dummy_cli_tool.ref_tool.find_any()
        # 电机设为上电
        self.dummy_driver.robot.set_enable(True)

    def init_pub_sub(self):
        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/dummy_arm/controller/pose_command',
            self.end_pos_callback,
            10
        )

        self.joint_sub = self.create_subscription(
            JointTrajectory,
            '/dummy_arm/controller/joints_command',
            self.joints_callback,
            10
        )

        # Publishers
        self.j6_pose_pub = self.create_publisher(PoseStamped, '/dummy_arm/current/pose/j6', 10)
        self.tool0_pose_pub = self.create_publisher(PoseStamped, '/dummy_arm/current/pose/tool0', 10)
        self.joint_pub = self.create_publisher(JointState, '/dummy_arm/current/joint_states', 10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timer (50 hz)
        self.timer = self.create_timer(0.02, self.call_poll_position)
        
    def call_poll_position(self):
        # 读取当前关节角度
        self.current_joints = self.current_joints_callback()
        # 发布当前关节角度
        self.handle_current_joints(self.current_joints)
        
        # 读取当前末端位置
        current_end_pose = self.current_end_callback()
        # 发布当前末端位置
        self.handle_current_end_pose(current_end_pose)


    def current_joints_callback(self):
        # 獲取與發佈当前关节角度
        j1 = self.dummy_driver.get_current_joint(0)
        j2 = self.dummy_driver.get_current_joint(1)
        j3 = self.dummy_driver.get_current_joint(2)
        j4 = self.dummy_driver.get_current_joint(3)
        j5 = self.dummy_driver.get_current_joint(4)
        j6 = self.dummy_driver.get_current_joint(5)
        return np.array([j1, j2, j3, j4, j5, j6])

    def current_end_callback(self):
        x = self.dummy_driver.get_current_pose6D(0)
        y = self.dummy_driver.get_current_pose6D(1)
        z = self.dummy_driver.get_current_pose6D(2)
        roll = self.dummy_driver.get_current_pose6D(3)
        pitch = self.dummy_driver.get_current_pose6D(4)
        yaw = self.dummy_driver.get_current_pose6D(5)
        return np.array([x, y, z, roll, pitch, yaw])

    def handle_current_joints(self, current_joints):
        try:
            joint_state = JointState()
            joint_state.header.stamp = self.get_clock().now().to_msg()
            joint_state.name = self.joint_names
            joint_state.position = current_joints.tolist()

            self.joint_pub.publish(joint_state)

            self.get_logger().info(
                f'Publish JPOS: j1: {current_joints[0]:.2f}, j2: {current_joints[1]:.2f}, '
                f'j3: {current_joints[2]:.2f}, j4: {current_joints[3]:.2f}, '
                f'j5: {current_joints[4]:.2f}, j6: {current_joints[5]:.2f}'
            )
        except Exception as e:
            self.get_logger().error(f'Parse joint error: {str(e)}')
    
    def handle_current_end_pose(self, current_end_pose):
        try:
            # 提取並換算位置與姿態（mm → m）
            x = current_end_pose[0] / 1000.0
            y = current_end_pose[1] / 1000.0
            z = current_end_pose[2] / 1000.0
            roll = current_end_pose[3]
            pitch = current_end_pose[4]
            yaw = current_end_pose[5]

            # 轉為弧度
            rolld = np.radians(roll)
            pitchd = np.radians(pitch)
            yawd = np.radians(yaw)

            # 轉換為四元數 [x, y, z, w]
            quat = R.from_euler('xyz', [rolld, pitchd, yawd]).as_quat()

            stamp = self.get_clock().now().to_msg()

            # --- Publish J6 PoseStamped ---
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

            # --- Publish J6 TF ---
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

            # --- Tool0 Pose = J6 Pose + offset ---
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

            # --- Tool0 TF (relative to joint6) ---
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

            # --- Camera TF (relative to joint6) ---
            camera_tf = TransformStamped()
            camera_tf.header.stamp = stamp
            camera_tf.header.frame_id = "joint6"
            camera_tf.child_frame_id = "camera_frame"
            camera_tf.transform.translation.x = -0.04
            camera_tf.transform.translation.y = 0.02
            camera_tf.transform.translation.z = 0.025
            camera_tf.transform.rotation.x = 0.0
            camera_tf.transform.rotation.y = 0.0
            camera_tf.transform.rotation.z = 0.0
            camera_tf.transform.rotation.w = 1.0
            self.tf_broadcaster.sendTransform(camera_tf)

            self.get_logger().info(
                f"Published LPOS: x={x:.3f}, y={y:.3f}, z={z:.3f}, roll={roll:.1f}, pitch={pitch:.1f}, yaw={yaw:.1f}"
            )
        except Exception as e:
            self.get_logger().error(f"Parse pose error: {str(e)}")
    
    def reset_pub_sub(self):
        # --- 銷毀原有 pub/sub ---
        if hasattr(self, 'pose_sub') and self.pose_sub is not None:
            self.destroy_subscription(self.pose_sub)
            self.get_logger().info("Destroyed pose_sub")
        if hasattr(self, 'joint_sub') and self.joint_sub is not None:
            self.destroy_publisher(self.joint_sub)
            self.get_logger().info("Destroyed joint_pub")
        
        if hasattr(self, 'j6_pose_pub') and self.j6_pose_pub is not None:
            self.destroy_publisher(self.j6_pose_pub)
            self.get_logger().info("Destroyed j6_pose_pub")
        if hasattr(self, 'tool0_pose_pub') and self.tool0_pose_pub is not None:
            self.destroy_publisher(self.tool0_pose_pub)
            self.get_logger().info("Destroyed tool0_pose_pub")
        if hasattr(self, 'joint_pub') and self.joint_pub is not None:
            self.destroy_publisher(self.joint_pub)
            self.get_logger().info("Destroyed joint_pub")

        if hasattr(self, 'tf_broadcaster') and self.tf_broadcaster is not None:
            self.tf_broadcaster = None
            self.get_logger().info("Destroyed tf_broadcaster")
        
        if hasattr(self, 'timer') and self.timer is not None:
            self.destroy_timer(self.timer)
            self.get_logger().info("Destroyed timer")
    
    def home_dummy(self):
        self.dummy_driver.robot.move_j(
            self.home_joints[0], self.home_joints[1], self.home_joints[2],
            self.home_joints[3], self.home_joints[4], self.home_joints[5])

    def stop_dummy(self):
        self.dummy_driver.robot.move_j(
            self.current_joints[0], self.current_joints[1], self.current_joints[2],
            self.current_joints[3], self.current_joints[4], self.current_joints[5])

    def reset_dummy(self):
        self.dummy_driver.robot.move_j(
            self.reset_joints[0], self.reset_joints[1], self.reset_joints[2],
            self.reset_joints[3], self.reset_joints[4], self.reset_joints[5])

    def end_pos_callback(self, msg: PoseStamped):
        # 1. 提取位置
        x = msg.pose.position.x
        y = msg.pose.position.y
        z = msg.pose.position.z

        # 2. 四元數轉歐拉角（Roll, Pitch, Yaw）
        qx = msg.pose.orientation.x
        qy = msg.pose.orientation.y
        qz = msg.pose.orientation.z
        qw = msg.pose.orientation.w
        r = R.from_quat([qx, qy, qz, qw])
        roll, pitch, yaw = r.as_euler('xyz', degrees=True)
        self.move_joints(np.array([x, y, z, roll, pitch, yaw]))

    def joints_callback(self, msg: JointTrajectory):
        if not msg.points:
            self.get_logger().warn("Received JointTrajectory with no points")
            return

        positions = msg.points[0].positions
        if len(positions) < 6:
            self.get_logger().warn(f"Expected 6 joint values, got {len(positions)}")
            return
        self.move_joints(np.array(positions))

    def move_joints(self, joints_deg):
        # joints_deg = self.rad_fix(joints_deg)
        self.dummy_driver.robot.move_j(joints_deg[0], joints_deg[1], joints_deg[2],
                                       joints_deg[3], joints_deg[4], joints_deg[5])
    
    def off_dummy(self):
        self.dummy_driver.robot.set_enable(False)
        self.dummy_driver = None
        self.get_logger().info("Dummy driver powered off")

def main(args=None):
    rclpy.init(args=args)
    node = dummy_controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # node.cleanup()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

