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
