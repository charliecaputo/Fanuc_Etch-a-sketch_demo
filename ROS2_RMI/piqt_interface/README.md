# FANUC RMI DEMO PC

### This is the PC side of this demo
I pivoted from the main branch as I was required to downgrade from Jazzy to humble. This Branch uses RMI directly rather than going through the fanuc driver and joint_traj_controllers. I tried but I could not for the life of me get smooth motion out of it on humble. The requiremnts are pretty much the same as before so you can look at the main branch for instructions on that.\

Once you have things ready you need to run 2 codes from this branch and the only code on the RPI branch

here you run:
1. ros2 run piqt_interface robot_connection
2. ros2 run piqt_interface test_rmi

The first establishes a connection to the robot and acts as the middle man between nodes and the robot. I do it this way so that when an hmi is built later, you can have the hmi run this and each button can be a different code. We hate long codes here.

The second is the main node for now. It takes the encoder position and creates an json package to send to the robot.

This is built directly off of John Castellani's code.