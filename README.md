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
- Hardware: Dummy arm with serial interface
---
- ROS2 Humble 或更新版本
- Python 3.8 及以上版本
- 硬件：具备串口接口的 Dummy 机械臂

## Usage | 使用方法
1. Clone this repository to your ROS2 workspace `src/` folder.
2. Clone Serial lib from https://github.com/ZhaoXiangBox/serial to the `src/` folder
3. Run `colcon build` to compile.
4. Launch the driver using 
```
source install/setup.bash
ros2 run dummy dummy_controller --ros-args --params-file ./src/dummy/config/config.yaml 
```
5. Publish messages to control the robot arm.
---
1. 克隆本仓库至 ROS2 工作区的 `src/` 目录下。
2. 克隆Serial仓库（https://github.com/ZhaoXiangBox/serial）到同一个`src/`目录
3. 使用 `colcon build` 进行编译。
4. 通过以下命令启动驱动
```
source install/setup.bash
ros2 run dummy dummy_controller --ros-args --params-file ./src/dummy/config/config.yaml 
```
5. 发布消息控制机械臂动作。


## Acknowledgements | 致谢
[1] https://github.com/peng-zhihui/Dummy-Robot

[2] https://gitee.com/switchpi/dummy