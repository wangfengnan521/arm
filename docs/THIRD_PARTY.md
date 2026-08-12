# 第三方与厂商依赖

本仓库只保存项目源码和运行所需的 X5A description 资源，不复制完整厂商 SDK、RealSense 或 MoveIt 仓库。

## ARX_X5

- 来源：<https://github.com/ARXroboticsX/ARX_X5>
- 本机验证基线 commit：`c783287` (`update`)
- 许可证：BSD 3-Clause
- 外部提供：`arx5_arm_msg`、`arx_x5_controller`、`arm_control`、ARX SDK/CAN 工具
- 本仓库 `src/X5A` 含厂商 X5A URDF/mesh 资源及项目使用的 MoveIt TCP wrapper。
- 厂商许可证副本见 `src/X5A/LICENSE.ARX_X5`。

构建和运行前应先按厂商文档编译 `ARX_X5/ROS2/X5_ws`，然后 source 其 `install/setup.bash`。

## 其它依赖

- ROS 2 Humble：<https://docs.ros.org/en/humble/>
- MoveIt 2：系统 apt package，验证版本 2.5.9
- Intel RealSense ROS：系统 apt package，验证版本 4.58.2
- librealsense：验证版本 2.58.2
- OpenCV / cv_bridge：系统 ROS package

这些依赖均不 vendoring 到本仓库。

