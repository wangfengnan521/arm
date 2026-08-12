# ARX X5A ROS 2 视觉抓取放置系统

## 一键视觉抓取

机械臂、RealSense、CANable2 正确连接后：

```bash
cd <workspace>
./run_x5a_vision_pick.sh
```

选择方块颜色（默认 `red`）：

```bash
./run_x5a_vision_pick.sh --color red
./run_x5a_vision_pick.sh --color white
./run_x5a_vision_pick.sh --color orange
```

仅视觉测试：

```bash
./run_x5a_vision_pick.sh --vision-only
```

只规划不执行：

```bash
./run_x5a_vision_pick.sh --dry-run --color orange
```

脚本会自动 source ROS/厂商/项目环境，复用健康的既有节点，并依次检查 CAN、真实关节反馈、MoveIt actions、RealSense 消息、Eye-to-Hand TF 和稳定视觉坐标。默认只执行一次，放置并回 Home 后退出。详细日志保存在 `logs/run_YYYYMMDD_HHMMSS/`。

## 项目简介

这是一个在真实 ARX X5A 上运行的 ROS 2 Humble 项目，使用 MoveIt 2、Intel RealSense RGB-D 和 Eye-to-Hand 手眼标定，实现视觉定位、抓取、视觉目标放置及回 Home。

当前稳定版本已经在两个 XY 距离约 0.174 m 的明显不同位置完成真实视觉抓取：

```text
REAL MOVEIT EXECUTION: PASS
FIXED-POSE PICK AND PLACE: PASS
EYE-TO-HAND CALIBRATION: PASS
VISION PICK AND PLACE: PASS
VISION CLOSED LOOP VERIFIED
```

## 已实现功能

- X5A `robot_description`、URDF/Xacro 和 mesh
- MoveIt 2 OMPL planning、Planning Scene 和真机 ExecuteTrajectory
- `RobotStatus` → `/joint_states` 状态反馈
- `FollowJointTrajectory` → ARX `RobotCmd` 正式适配器
- 标准 `control_msgs/action/GripperCommand` 夹爪控制
- 固定坐标 Pick & Place
- ChArUco Eye-to-Hand 标定与 TF 发布
- RealSense aligned RGB-D 红、白、橙三色方块独立定位
- 黑色海绵区域中心定位，作为可移动放置目标
- 视觉坐标冻结、Cartesian approach/lift/descend/retreat 和 Pose planning fallback
- 两个不同位置的真实视觉 Pick & Place 闭环验证

## 系统架构

正式真机执行链：

```text
RealSense
   |
   v
x5a_vision -- selected cube pose + /x5a_vision/box_pose
   |
   v
TF2 / Eye-to-Hand
   |
   v
x5a_pick_place
   |
   v
MoveIt 2 ExecuteTrajectory
   |
   v
FollowJointTrajectory + GripperCommand
   |
   v
x5a_moveit_official_adapter
   |
   v
ARX /arm_cmd (RobotCmd)
   |
   v
official X5Controller -> CAN -> X5A
```

反馈链：

```text
X5A -> CAN -> X5Controller -> /arm_status (RobotStatus)
    -> x5a_moveit_official_adapter -> /joint_states -> MoveIt / RViz
```

`x5a_control_bridge` 是项目早期 bridge/测试实现，保留用于历史和实验参考。稳定版真机运行必须使用 `x5a_moveit_official_adapter`，禁止两个 bridge/adapter 同时运行；`/arm_cmd` publisher count 必须严格等于 1。

## 软件环境

- Ubuntu 22.04.5 LTS (x86_64)
- ROS 2 Humble
- MoveIt 2.5.9
- Intel RealSense D435i，librealsense 2.58.2
- `realsense2_camera` 4.58.2
- OpenCV / `cv_bridge` 3.2.1
- Python 3.10（ROS 2 Humble 系统 Python）

## ROS 2 packages

| Package | 用途 |
|---|---|
| `X5A` | 厂商 X5A description/mesh 与本项目 MoveIt TCP wrapper |
| `x5a_moveit_config` | MoveIt 2 模型、规划参数、控制器映射、真机 bringup |
| `x5a_moveit_official_adapter` | `/arm_status` 状态适配，FJT/Gripper action 到官方 `/arm_cmd` |
| `x5a_pick_place` | 固定坐标和视觉抓放状态机、Planning Scene、执行与回 Home |
| `x5a_vision` | HSV + RGB-D + TF2 的红/白/橙方块及黑色海绵中心定位 |
| `x5a_handeye` | ChArUco 检测、采样、Eye-to-Hand 求解、验证与 TF 发布 |
| `x5a_control_bridge` | 早期实验 bridge；稳定真机链不启动 |

厂商 `arx5_arm_msg`、`arx_x5_controller` 和 SDK 保持为外部依赖，详情见 [docs/THIRD_PARTY.md](docs/THIRD_PARTY.md)。

## 获取依赖与编译

先安装 ROS 2 Humble、MoveIt 2、RealSense ROS wrapper 和 OpenCV ROS packages，并按厂商文档准备 ARX 工作区：

```bash
git clone https://github.com/ARXroboticsX/ARX_X5.git ~/repos/arx/ARX_X5
cd ~/repos/arx/ARX_X5/ROS2/X5_ws
source /opt/ros/humble/setup.bash
colcon build
```

克隆本项目并编译：

```bash
git clone https://github.com/wangfengnan521/arm.git ~/arx/arm
cd ~/arx/arm
source /opt/ros/humble/setup.bash
source ~/repos/arx/ARX_X5/ROS2/X5_ws/install/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## CAN

真机验证使用 CANable2、slcan、`can1`、1 Mbps（`slcand -s8`）。设备 udev alias 为 `/dev/arxcan1` 时：

```bash
cd ~/arx/arm
./scripts/setup_can_x5a.sh
```

脚本不会保存 sudo 密码，并会拒绝为同一接口启动第二个 `slcand`。启动后应看到 `can1` 为 `UP`、`ERROR-ACTIVE`。不要运行多个 `slcand` 或看门狗循环。

## 启动

以下命令分别在独立终端运行。真实运动前确认工作区无人、急停可用、CAN 正常，并确保只有一个 `X5Controller` 和一个正式适配器。

### 1. 官方 X5A 控制器

```bash
source /opt/ros/humble/setup.bash
source ~/repos/arx/ARX_X5/ROS2/X5_ws/install/setup.bash
ros2 launch arx_x5_controller open_single_arm.launch.py
```

### 2. X5A + MoveIt + RViz

```bash
source /opt/ros/humble/setup.bash
source ~/repos/arx/ARX_X5/ROS2/X5_ws/install/setup.bash
source ~/arx/arm/install/setup.bash
ros2 launch x5a_moveit_config x5a_real_moveit.launch.py \
  start_driver:=false start_adapter:=true use_rviz:=true
```

也可以由该 launch 启动官方 driver，此时必须保证外部 `X5Controller` 未运行，并使用 `start_driver:=true`。

### 3. RealSense / Eye-to-Hand TF / Vision

```bash
source /opt/ros/humble/setup.bash
source ~/arx/arm/install/setup.bash
ros2 launch x5a_vision vision.launch.py start_camera:=true
```

若相机已经运行，使用 `start_camera:=false`，不要启动第二个 RealSense 节点。

### 4. Pick & Place

先只规划：

```bash
ros2 launch x5a_pick_place pick_place.launch.py \
  plan_only:=true vision_enabled:=true
```

确认 `/arm_cmd` publisher count 为 1、三个 action 齐全并完成现场安全检查后，真机执行：

```bash
ros2 launch x5a_pick_place pick_place.launch.py \
  plan_only:=false vision_enabled:=true
```

正式链禁止直接发布 `/arm_cmd`、禁止同时启动 `x5a_control_bridge`、禁止绕过 MoveIt 插值关节轨迹。

## Eye-to-Hand calibration

当前实机标定配置从 `src/x5a_handeye/config/handeye_result.yaml` 和 `board.yaml` 读取：

- Board：ChArUco，`DICT_4X4_50`，5 x 7
- square：0.020 m
- marker：0.014 m
- Solver：PARK
- Samples：33
- Hold-out mean：7.67 mm
- Hold-out max：14.07 mm
- 变换：`T_base_camera_color_optical`
- 标定 verdict：PASS

运行时仅额外发布 `base_link -> camera_link`；RealSense 自己发布 `camera_link -> ... -> camera_color_optical_frame`。禁止为 `camera_color_optical_frame` 创建第二个 parent。四元数整体变号表示同一个旋转。

## Visual Pick & Place

稳定版使用 15 帧 median/std 过滤；检测稳定后同时冻结所选方块和黑色海绵中心，本轮运动期间不再跟随视觉漂移。红色视觉抓放基线曾在两个不同位置完成闭环验证：

```text
position 1: base XY ~= [0.2140, 0.1755] m
position 2: base XY ~= [0.2978, 0.3279] m
distance:             ~= 0.174 m
```

两次均由不同视觉坐标完成抓取并放到固定 place pose：

```text
VISION PICK AND PLACE: PASS
VISION CLOSED LOOP VERIFIED
```

## 参数修改

- 物块尺寸、HSV、深度和 workspace：`src/x5a_vision/config/vision.yaml`
- 抓取 offset、物块尺寸、table、place pose、速度：`src/x5a_pick_place/config/pick_place.yaml`
- FollowJointTrajectory/夹爪适配器频率与保护：`src/x5a_moveit_official_adapter/config/adapter.yaml`
- 手眼外参：`src/x5a_handeye/config/handeye_result.yaml`
- ChArUco board：`src/x5a_handeye/config/board.yaml`
- MoveIt joint limits/kinematics/controller：`src/x5a_moveit_config/config/`

当前实机验证的抓取 `z_offset` 为 0.040 m。除非有标定证据，不应通过修改手眼 TF 凑抓取结果。

## 已知限制

- 当前检测器主要针对已知 HSV 范围的小型红色方块，不是通用物体识别系统。
- `tool0` 是当前标定和实际使用的 TCP。
- X5A joint limits 中部分来自 X5 software limits，并结合本机可达性做了限制；不等于完整机械范围认证。
- 当前正式 bridge 是项目实现的 `FollowJointTrajectory` / `GripperCommand` 到 ARX `RobotCmd` 适配器。
- 当前不是完整的 `ros2_control` `HardwareInterface`。
- 三色方块和可移动黑色海绵目标已经完成检测、坐标冻结和完整 dry-run；部分位置已完成真实放置。
- 携物跨侧移动到部分盒子位置时，`MOVE_PRE_PLACE` 仍可能被 OMPL 以 `Unable to sample any valid states for goal tree` 拒绝。当前实现会预求解一个 IK 关节分支，该分支规划失败后尚未继续搜索其他 Pose Goal IK 分支；这是当前待修问题，不代表视觉坐标错误。
- 相机 serial number 在 `x5a_vision/launch/vision.launch.py` 中按本机 D435i 配置；更换相机时需更新或参数化。
- 真机操作必须由熟悉机械臂、CAN 和急停流程的人员现场监护。
