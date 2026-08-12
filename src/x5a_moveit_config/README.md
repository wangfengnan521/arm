# x5a_moveit_config

MoveIt 2 planning and verified real-robot execution configuration for ARX X5A.

## Hardware status

- Real MoveIt ExecuteTrajectory is verified through `x5a_moveit_official_adapter`.
- Joint limits include X5 software limits and project commissioning constraints; they are not a full mechanical-range certification.
- `tool0` is the TCP used by the verified Eye-to-Hand and pick/place setup.
- Never run `x5a_control_bridge` together with the official adapter.

## Launch

```bash
source /opt/ros/humble/setup.bash
source ~/arx/arm/install/setup.bash
ros2 launch x5a_moveit_config demo.launch.py
```
