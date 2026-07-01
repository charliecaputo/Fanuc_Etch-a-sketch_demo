# Etch-a-sketch ROS2 FANUC Demo

## About
This repo is for a project where we are using a crx-10ia as an etch-a-sketch (EAS) controlled by a raspberry pi. The pi is connected to 2 as5600 magnetic encoders that are used to move it in the x and y direction of the drawing plane. For testing I used the keyboard to move the robot since I didn't have access to the encoders yet, so there are also keyboard controls in this repo. This repo also has drivers from fanuc and the robot descriptions from fanuc. you can use these for this project but they are modified a bit so be careful with updating those from source (see below for modified codes that should be copied/saved incase of updates from source).

## Current State
Currently the code is running off the robot hmi. One encoder works (for x direction) since I don't have the other encoder yet. There are 2 jog pages (cartesian and joint) both work and have a speed slider to adjust the speed of movemnent from the robot. There is also a manual initalization code wired to the home button on the hmi that initalizes to the center of the board with a ~1cm Z+(world) offset (VERIFY POSITION BEFORE RUNNING WITH THE REAL ROBOT).


## Dependencies
keyboard_teleop, servo_control = pynput (allows keybaord input).\
Enocder_test = smbus2 (allows i2c connection).\
These should be installable with ros "rosdep install --from-paths src --ignore-src -r -y". If not they can be installed with "sudo apt install python3-\<dependecy name\>"

## Modifications and Codes to Copy
1. fanuc_description/fanuc_crx_description/urdf/crx10ia_urdf_macro.xacro
2. fanuc_description/fanuc_crx_description/meshes/D100124_demo/
3. /fanuc_driver/fanuc_hardware_interface/config/ros2_controllers.yaml (only lines 38 and 40 need to be copied from the controller)

4. /fanuc_driver/fanuc_moveit_config/config/EAS_servo.yaml


## Instructions and Button funtions 
1. cd into /robot_hmi
2. python3 hmi_app.py

The HMI should pop up at this point\
**Buttons (Main Page):**
1. Enable Robot: This runs the launch code found in test_py which establishes connection and starts ros2 controllers for the crx10ia (Currently the simulated robot)
2. Home Robot: Homes the robot 
3. Encoder Teleop: The current functional code. Uses keyboard for y axis and encoder for x axis (still waiting on second encoder)
4. Ship: Moves the robot in and out of the shipping position based on its current location (joint states).
5. E-Stop: Shuts down everything but the app. I am debating on getting rid of the abort button and just making this kill everything but the launch file. The estop latches and to reset it you must hold it down for 3 seconds.
6. "X" in corner: Same as estop but also shuts down the app
7. \"-" next to x: Minimizes the page

8. Manual Control: This takes you to the cartesian jog page

> **Note:** Codes should shutdown when another is launched. i.e. if you click the "Encoder Teleop" while "Home Robot" is already running, "Home Robot" will shutdown and vice versa. This also includes when you switch to and from manual control as manual control has its own code. 

**Buttons (Cartesian Jog Page):**
1. Back: Takes you to Main Page
2. Joint: Takes you to the Joint jog page
3. 12 buttons in the middle: Jogs the robot in the respective cartesian coordinate.
4. Slider at the bottom: Controls the speed as a percentage

**Buttons (Joint Jog Page):**
1. Back: Takes you to Main Page 
2. Cartesian: Takes you to the Cartesian jog page
3. 12 buttons in the middle: Jogs the robot in the respective joint coordinate.
4. Slider at the bottom: Controls the speed as a percentage.
> **Note:** Speed carries between the jog pages and does NOT reset unless app is restarted. Does NOT affect codes run from main page.
