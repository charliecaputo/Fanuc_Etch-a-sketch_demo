First let me say sorry for naming this pkg as test_py. I'm too deep into this now and too lazy to go through and change it all. The code works and thats all that really matters. 

This pkg houses the main code for this specific project created by your favorite (and best) intern.

If you dont want to use the hmi you can still run the codes uing ros2 run and launch:
1. ros2 launch test_py fac_moveit_test.launch.py use_mock:=true\
>**Note:** Use_mock is for simulated robot. This is the main launch file for the project
2. ros2 run test_py \<node of choice\>
- servo_control: Current working version of "Encoder_Teleop". requires "ros2 run fanuc_crx_xy_demo encoder_test.py" to be running in order to work
- manual_init: Homes the robot
- keyboard_tool_teleop and cartesian_kb_teleop: keyboard control of robot using I = +x, K = -x, J = +y, L= -y, no roll pitch yaw.
>**Note:** I don't remeber what the difference is between these.
- encoder_read: A copy of fanuc_crx_xy_demo encoder_test.py. I wanted to bring it to test_py and have it self contained but for some reason i cant get it to use the yaml in test_py so it pulls from fanuc_crx_xy_demo.
- jog_listener_node: listens to button inputs from hmi. This won't work unless the hmi is running unless you use terminal (or a new code) to publish commands to /hmi/jog_command.
- ship_pos: moves the robot in and out of shipping position. it does this based on the joint states of the robot. you need to press the hmi button or publish to /ship_pose which is a boolean
