# X5A robot_description (ROS 2)

Vendor ARX X5A URDF and meshes packaged for ROS 2 Humble visualization.

Source model: `ARX_Model/X5/X5A` (SolidWorks URDF exporter). Joint axes and origins are unchanged.

## Build

```bash
cd ~/arx/arm
source /opt/ros/humble/setup.bash
colcon build --packages-select X5A
source install/setup.bash
```

## Display (pure model)

```bash
ros2 launch X5A display.launch.py
```
