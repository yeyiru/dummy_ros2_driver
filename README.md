# Dummy Ros2 Driver | 稚晖君/木子Dummy机械臂ROS2驱动程序

##  Introduction | 简介
This is the unofficial ROS2 driver for the open-source robotic arm designed by Zhihui Jun and Muzi. The driver enables seamless integration with the ROS2 ecosystem, allowing users to control and interact with the robotic arm through joint angle or end-effector commands.

这是稚晖君与木子共同开源的机械臂方案所设计的ROS2 非官方驱动程序。该驱动支持通过关节角度或末端执行器指令与机械臂进行交互，并可与 ROS2 生态系统无缝集成。


## Features | 功能特点

- Support for joint control and pose control
- Easy to integrate with simulation environments like Gazebo or Isaac Sim
- Lightweight and modular design
---
- 支持关节控制与位姿控制
- 可轻松集成至 Gazebo 或 Isaac Sim 等仿真环境
- 轻量化、模块化设计
- 基于 ROS2 的 rclpy 实现，便于使用与拓展

## Project Structure | 项目结构
```
dummy_ros2_driver/
├── launch/                 # 启动文件 TODO
├── config/                 # 配置文件（如URDF路径、参数）
├── src/                    # 驱动核心代码
├── urdf/                   # 机械臂描述文件（URDF）TODO
└── README.md               # 项目说明文件（当前文件）
```

## Requirements | 运行依赖
- ROS2 Humble
- Python 3.8+
- ROS2 serial_driver (You can isntall it by `sudo apt-get install ros-humble-serial-driver`)
- Hardware: Dummy arm with serial interface
---
- ROS2 Humble 或更新版本
- Python 3.8 及以上版本
- ROS2 serial_driver (可以通过 `sudo apt-get install ros-humble-serial-driver` 安装)
- 硬件：具备串口接口的 Dummy 机械臂

## Usage | 使用方法
1. Clone this repository to your ROS2 workspace `src/` folder.
2. Run `colcon build` to compile.
3. Launch the driver using 
```
source install/setup.bash
# Only driver Dummy ARM
ros2 run dummy dummy_controller --ros-args --params-file ./src/dummy/config/config.yaml
# Simultaneously launch the dummy controller and USB camera
ros2 launch dummy dummy_usbcamera.launch.py
```
5. Publish messages to control the robot arm.
---
1. 克隆本仓库至 ROS2 工作区的 `src/` 目录下。
2. 使用 `colcon build` 进行编译。
3. 通过以下命令启动驱动
```
source install/setup.bash
# 單獨控制dummy手臂
ros2 run dummy dummy_controller --ros-args --params-file ./src/dummy/config/config.yaml 
# or
ros2 launch dummy dummy_controller.launch.py
# 如果需要啟動USB相機節點
ros2 launch dummy dummy_usbcamera.launch.py
```
5. 发布消息控制机械臂动作。

## 生命週期節點狀態
1. `ros2 launch dummy dummy_controller.launch.py` [Node started up] 啟動節點，讀取到config參數
2. `ros2 lifecycle set /dummy_controller configure` [Configure status] 打開串口、電機使能、手臂進入7字狀態
3. `ros2 lifecycle set /dummy_controller activate` [Activate status] 初始化發佈器訂閱器，開始讀取關節角度信息，開始接受外部控制指令
4. `ros2 lifecycle set /dummy_controller deactivate` [Decativate status] 停止讀取關節角度，銷毀發佈器、接收器
5. `ros2 lifecycle set /dummy_controller cleanup`[Inconfigure status] 手臂回到收回位置、電機關閉、關閉發佈器和接收器、關閉串口
6. `ros2 lifecycle set /dummy_controller shutdown`[Shutdown status] 手臂回到收回位置、電機關閉、關閉發佈器和接收器、關閉串口


## Acknowledgements | 致谢
[1] https://github.com/peng-zhihui/Dummy-Robot
[2] https://gitee.com/switchpi/dummy