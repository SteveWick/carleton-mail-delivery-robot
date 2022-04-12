# carleton-mail-delivery-robot
By Stephen Wicklund & Emily Clarke
This repository is for the 2021-2022 Capstone project titled Carleton University Mail Delivery Robot.

## Quick Start Guide
Follow these instructions to run the currently loaded software without any changes.

1. Ensure that the raspberry pi is powered through an external battery bank. The raspberry Pi 4 uses a USB type C cable. Any standard portable battery bank will work.

![image](https://user-images.githubusercontent.com/24395948/163032276-5105b304-591c-4786-bd1e-547c2c962ddf.png)


2. The Robot should appear as above, the cable from the raspberry Pi to the robot should be connected, as well as the IR Sensors.


4. Connect to the headless raspberry pi through ssh, username and password are labeled on the board.


6. Setup ROS with the following two commands


`source /opt/ros/foxy/setup.bash`


`source ~/create_ws/install/setup.bash`

5. Ensure the iRobot Create 1 or iRobot Create 2 is on. Press the power button, it should beep if the battery is charged and the green light will turn on.

5. Run the ROS package with the following command:


`ros2 launch mail_delivery_robot robot.launch.py`


Or the iRobot Create 2 with:


`ros2 launch mail_delivery_robot robot.launch.py 'robot_model:=CREATE_2'`


6. The robot will start in wall following behavior and should follow a wall. If no beacons are found it will not turn.
