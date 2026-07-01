# Etch-a-sketch ROS2 FANUC Demo

## About
This repo is for a project where we are using a crx-10ia as an etch-a-sketch (EAS) controlled by a raspberry pi. The pi is connected to 2 as5600 magnetic encoders that are used to move it in the x and y direction of the drawing plane. For testing I used the keyboard to move the robot since I didn't have access to the encoders yet, so there are also keyboard controls in this repo. This is setup using the fanuc drivers and description repos which are slightly modified (see Modifications section). This repo is currently running on Ubuntu 24.04 ros2 Jazzy.

## Dependencies
keyboard_teleop, servo_control = pynput (allows keybaord input).\
Enocder_test = smbus2 (allows i2c connection).\
These should be installable with ros "rosdep install --from-paths src --ignore-src -r -y". If not they cna be installed with "sudo apt install python3-\<dependecy name\>"

## Modifications
Go to:
fanuc_driver > fanuc_hardware > config > ros2_controllers.yaml\
and change:
  * allow_nonzero_velocity_at_trajectory_end: true
  * stopped_velocity_tolerance: 0.05\
  reason: if you want smooth movement of the robot then you must make this non zero. this is the equivalent of making a cnt movement vs fine movement

In the config folder from this repo, copy EAS_servo.yaml into fanuc_driver > fanuc_moveit_config > config


## Instructions to run
1. ros2 launch test_py fac_moveit_test.launch.py use_mock:=true\
  note: use_mock is for simulated robot.
2. ros2 run test_py manual_init\
  note: after running this and robot is in position kill this node.
3. ros2 run test_py \<node of choice\>