# piqt_interface

`piqt_interface` is a ROS 2 Python package that provides a PyQt GUI for FANUC RMI (Remote Motion Interface).

It is designed for testing, debugging, and manual operation of FANUC RMI commands without writing custom scripts each time.

## What This Package Does

- Connects to a FANUC controller over RMI using the standard handshake flow
- Sends RMI command and motion packets from a GUI
- Dynamically builds command input forms from packet definitions
- Tracks motion `SequenceID` values and queued commands
- Shows robot responses and status in real time

## Key Features

- Connection controls: Connect, Disconnect, Initialize, Reset, Abort
- Command browser: Select from supported RMI commands and instructions
- Dynamic parameter UI: Fields change based on selected packet type
- Sequence tracking: Last sent, last received, and next sequence values
- Background receive thread: Keeps GUI responsive while waiting for packets

## Requirements

- ROS 2 workspace with `ament_python`
- Python dependencies used by this package:
	- `rclpy`
	- `PyQt5`
	- `std_msgs`
- Network access to the robot controller
- FANUC controller configured for RMI and reachable on the handshake port (`16001`)

## Build and Run

From your ROS 2 workspace root:

```bash
colcon build --packages-select piqt_interface
source install/setup.bash
ros2 run piqt_interface rmi_interface
```

## How to Use

1. Launch the GUI.
2. Enter the robot IP address (or leave blank to use default `192.168.1.100`).
3. Click **Connect**.
4. Click **Initialize**.
5. Select a command from the dropdown.
6. Fill in required fields.
7. Click **Send Command**.
8. Watch **Responses** and **Queued Packets** for feedback and completion.

## Command Support

The package includes packet definitions for common FANUC RMI operations, including:

- Robot control (`FRC_Initialize`, `FRC_Reset`, `FRC_Abort`, `FRC_GetStatus`)
- Motion instructions (joint, linear, circular, spline, waits, call)
- Registers and variables
- IO read/write
- User frame and tool frame read/write
- Payload and speed override commands

Commands are defined in the packet registry and automatically appear in the GUI command list.

## Notes

- Motion packets are queued and tracked by `SequenceID`.
- The interface sends and receives JSON over TCP sockets.
- This package is focused on operator/testing workflows, not autonomous motion planning.


# FANUC RMI DEMO PC

### This is the PC side of this demo
I pivoted from the main branch as I was required to downgrade from Jazzy to humble. This Branch uses RMI directly rather than going through the fanuc driver and joint_traj_controllers. I tried but I could not for the life of me get smooth motion out of it on humble. The requiremnts are pretty much the same as before so you can look at the main branch for instructions on that.\

Once you have things ready you need to run 2 codes from this branch and the only code on the RPI branch

here you run:
1. ros2 run piqt_interface robot_connection
2. ros2 run piqt_interface test_rmi
3. ros2 run test_py encoder_read
**NOTE**: make sure 3 is run on the pi.   

The first establishes a connection to the robot and acts as the middle man between nodes and the robot. I do it this way so that when an hmi is built later, you can have the hmi run this and each button can be a different code. We hate long codes here.

The second is the main node for now. It takes the encoder position and creates an json package to send to the robot.

This is built directly off of John Castellani's RMI code.
