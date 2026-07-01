# fanuc_crx_xy_demo

ROS 2 Python package for a Raspberry Pi 5 demo that reads two AS5600 magnetic encoders through a TCA9548A I2C multiplexer and publishes an X/Y target pose for later integration with the FANUC ROS 2 driver stack.

## What this package does
- `dual_as5600_mux_node`: reads two AS5600 sensors on a TCA9548A and publishes `encoder_xy_mm` as `[x_deg, y_deg, x_mm, y_mm]`
- `xy_target_node`: converts encoder feedback into `geometry_msgs/PoseStamped` on `demo_target_pose`
- `fanuc_moveit_target_bridge_example`: placeholder bridge node that logs incoming target poses

## Important
This package intentionally stops short of commanding the robot directly. Validate encoder scaling, TCP, frames, limits, payload, and safety before replacing the bridge node with a real motion execution path.

## Suggested dependencies outside rosdep
Install SMBus/I2C tools on Ubuntu:

```bash
sudo apt update
sudo apt install -y python3-smbus i2c-tools
pip3 install smbus2
```

## Build
From the workspace root:

```bash
colcon build --symlink-install
source install/setup.bash
```

## Run
```bash
ros2 launch fanuc_crx_xy_demo fanuc_crx_xy_demo.launch.py
```
