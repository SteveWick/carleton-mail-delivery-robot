# Carleton University Mail Delivery Robot
By Stephen Wicklund & Emily Clarke

Contact: stevewicklund@gmail.com

This repository is for the 2021-2022 Capstone project titled Carleton University Mail Delivery Robot.

## Overview

*This project has many components and it is highly recommended to review the Project Report (in the documentation/FinalReport directory) and read through the github wiki.*

The project is comprised of a hardware setup built on the iRobot Create 1 and iRobot Create 2, a single mono-repository (here) and a extensive report detailing the project. The repository contains all the software written for this project in one place. It primarily utilizes [ROS 2 - Foxy](https://docs.ros.org/en/foxy/index.html) with the [create_autonomy](https://github.com/AutonomyLab/create_robot/tree/foxy) library to drive the robot.

To set up the robot software from stratch follow the instructions [here](https://github.com/SteveWick/carleton-mail-delivery-robot/wiki/Robot-Set-up-ROS2).


## Getting Started

Before beginning work on this project it is highly recommended to research ROS and understand how nodes work and communicate. It's advise to review these [tutorials](https://docs.ros.org/en/foxy/Tutorials.html).

The ROS package is contained in the `mail_delivery_robot` directory and can be built and installed from there. To understand the `mail_delivery_robot` package it is recommended to review the report.

![image](https://user-images.githubusercontent.com/24395948/163036269-65eed720-d5c1-4cd5-9d57-b29a84a874de.png)

The `mail_devliery_robot` is composed of the following nodes and topics. (cmd_vel is the link to the create_autonomy package). The entire package centers around the robot_driver node which uses a state machine. Details of the state machine can be found in the report in section 5.1.4 Wall Following Navigation.

The final chapter of the report contains Reflections. This chapter will explain incomplete work, components which are not functioning properly and lessons learned from the previous group. **It is highly recommneded to review this chapter.**


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
