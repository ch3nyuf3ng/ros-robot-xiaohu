# 机器人小护 xiaohu_robot

机器人小护是一个基于 ROS 开发的用于执行简单医护任务的机器人，旨在缓解医疗资源紧缺的问题。机器人小护可以部署在家庭、疗养院、医院等场景，实现远程诊疗、送药等服务。该软件包实现了对机器人小护的总控 ROS 节点和辅助功能 ROS 节点，可以通过订阅 ROS 话题发送消息命令机器人执行相应的任务。该软件包还提供了必要的仿真模型、配置文件、节点加载脚本、测试脚本、依赖安装脚本等。

## 软件包结构

- `include`: C++ 头文件。
- `launch`: 各个 ROS 节点的启动脚本和参数文件。 
- `lib`: 讯飞语音 API 的静态库文件。
- `maps`: 保存地图文件。
- `msg`: ROS 节点间通信的消息类型。
- `src`: C++ 源代码。
- `tests`: 测试脚本。

## 安装

1. 安装 ROS Melodic: https://www.ros.org/install/
1. 安装如下依赖的 ROS 软件包：
    - [iai_kinect2](https://github.com/code-iai/iai_kinect2)
    - [rplidar_ros](https://github.com/Slamtec/rplidar_ros)
    - [wpb_home](https://github.com/6-robot/wpb_home)
    - [wpr_simulation](https://github.com/6-robot/wpr_simulation)
    - [serial](https://github.com/wjwwood/serial)
1. 安装温度传感器所需的依赖 Linux 系统的 I2C 库：
    - `sudo apt install i2c-tools`
1. 将 `xiaohu_robot` 添加到 catkin 工作环境的 `src` 目录下;
1. 使用 `catkin_make` 编译。

## 使用

1. 如果须在仿真环境下测试，请修改 `launch/main.launch` 中的 `is_real` 参数为 `false`；
1. 根据需要，修改 `launch/phy/params/compensation.yaml` 中的参数来校准摄像头的高度、角度等参数；
1. 启动机器人小护总控程序：`roslaunch xiaohu_robot main.launch`;
1. 运行相应的前后端。

## 测试

1. 启动机器人小护总控程序：`roslaunch xiaohu_robot main.launch`;
1. 运行 `tests/simulation/tasks.sh` 脚本，即可在仿真环境下测试机器人小护的功能。