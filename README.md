# Etch-a-sketch ROS2 FANUC Demo Version 1

## About
This repo is for a project where we are using a crx-10ia as an etch-a-sketch (EAS) controlled by a raspberry pi. The pi is connected to 2 as5600 magnetic encoders that are used to move it in the x and y direction of the drawing plane. For testing I used the keyboard to move the robot since I didn't have access to the encoders yet, so there are also keyboard controls in this repo. This repo also has drivers from fanuc and the robot descriptions from fanuc. you can use these for this project but they are modified a bit so be careful with updating those from source (see below for modified codes that should be copied/saved incase of updates from source).

## Current State
Currently the code is running off the robot hmi. One encoder works (for x direction) since I don't have the other encoder yet. There are 2 jog pages (cartesian and joint) both work and have a speed slider to adjust the speed of movemnent from the robot. There is also a manual initalization code wired to the home button on the hmi that initalizes to the center of the board with a ~1cm Z+(world) offset (VERIFY POSITION BEFORE RUNNING WITH THE REAL ROBOT).


## Dependencies
keyboard_teleop = pynput (allows keybaord input).\
Enocder_test = smbus2 (allows i2c connection).\
These should be installable with ros "rosdep install --from-paths src --ignore-src -r -y".\
pip install pyqt6 \
sudo apt install ros-humble-moveit* -y

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
3. Encoder Teleop: The current functional code.
4. Ship: Moves the robot in and out of the shipping position based on its current location (joint states).
5. E-Stop: Kills all running codes except the launch file
6. "X" in corner: Same as estop but also shuts down the app and launch file
7. \"-" next to x: Minimizes the page
8. Manual Control: This takes you to the cartesian jog page

> **Note:** Codes should shutdown when another is launched. i.e. if you click the "Encoder Teleop" while "Home Robot" is already running, "Home Robot" will shutdown and vice versa. This also includes when you switch to and from manual control as manual control has its own code. 

**Buttons (Cartesian Jog Page):**
1. Back: Takes you to Main Page
2. Joint: Takes you to the Joint jog page
3. Free Drive: Takes you to Free Drive page. 
4. 12 buttons in the middle: Jogs the robot in the respective cartesian coordinate.
5. Slider at the bottom: Controls the speed as a percentage

**Buttons (Joint Jog Page):**
1. Back: Takes you to Main Page 
2. Cartesian: Takes you to the Cartesian jog page
3. Free Drive: Takes you to Free Drive page.
4. 12 buttons in the middle: Jogs the robot in the respective joint coordinate.
5. Slider at the bottom: Controls the speed as a percentage.
> **Note:** Speed carries between the jog pages and does NOT reset unless app is restarted. Does NOT affect codes run from main page.

**Buttons (Joint Jog Page):**
1. Back: Takes you to Main Page
2. Hold For Free Drive: Self explanatory. Hold it down and the robot should go into Free Drive or Manual Guided Teaching.
3. 🔒: Allows user to lock the Free Drive mode so that the button doesn't need to be held. Press it, then press the free drive. When either button is pressed the lock is disabled.
>**Note:** This function is untested on the real robot. It works by letting user press and hold the button which raises Flag 8. Make sure to enable the "Enabling Input" in the collaborative settings of the robot. When the button is released Manual Guided Teaching is disabled.

## Current Issues and Fixes
Ethernet connection:\
If ever the connection fails. Go to wired settings. See if it says connecting or connected. if it says connecting, click the gear icon, click IPv4. then select manual. lastly make address = 192.168.1.5 and subnet mask = 255.255.255.0. then apply.\
>**Note:** YOU WIll need to run: sudo ip link set eth0 down and then sudo ip link set eth0 up
- add it to launch script 

## Videos
[Simulation and HMI](https://www.youtube.com/watch?v=KgPVzGzSmQw)


# Version 2 
Hey John,\ 
sorry this project is left in kind of a meesy state as I didn't really have time to build it up in the last couple weeks. Currently the encoder control of the robot works but only when running maunally (not through the hmi). The HMI very partially works. The ship pose button and the home button works but nothing else does. Unfortunatly the whole hmi will likely need to be reworked no that we are using rmi directly. The main branch of the code works for ros2 jazzy and the PC and RPI branches work on Humble. see the read me in PIQT_interface for how to run it the encoder stuff. Best of luck!
