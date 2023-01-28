#!/usr/bin/env python

# @author: Simon Yacoub, Devon Daley and Jacob Charpentier (built on top of previous year's work)

# SUBSCRIBER:   String object from 'actions' node
# PUBLISHER:    Twist object to 'cmd_vel' node
#               null object to 'dock' node
import math
import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time
import csv
import os
# This script is meant to take all the action decisions from our reasoner and publish them to the roomba (via cmd_vel)


# ~~~~ Load overrides ~~~~
def loadNumberOverrides():
    magicNumbers = {}
    ROOT_DIR = os.getcwd()
    with open(f'{ROOT_DIR}/src/carleton-mail-delivery-robot/mail_delivery_robot/magicNumbers.csv') as csvfile:
        reader = csv.reader(csvfile, delimiter=",")
        for row in reader:
            magicNumbers[row[0]] = row[1]
    return magicNumbers


magicNumbers = loadNumberOverrides()

class ActionTranslator(Node):
    def __init__(self):
        super().__init__('action_translator')
        self.drivePublisher = self.create_publisher(Twist, 'cmd_vel', 2)
        #unimplemented docking behaviour 
        self.undockPublisher = self.create_publisher(Empty, 'dock', 1)
        self.dockPublisher = self.create_publisher(Empty, 'undock', 1)
        
        self.subscription = self.create_subscription(String, 'actions', self.decodeAction, 10)

    # Decode and execute the action
    def decodeAction(self, data):
        action = str(data.data)
        self.get_logger().info(action)
        emptyMessage = Empty

        target_distance, current_distance, current_angle = action.split(":")
        time_interval = float(magicNumbers['TIMER_PERIOD'])

        message = Twist()
        message.angular.z = angular_speed(current_angle, current_distance, target_distance, time_interval)
        message.linear.x = float(magicNumbers['FORWARD_X_SPEED']) 
        
        # Get the parameters
        #(drivePublisher, dockPublisher, undockPublisher) = args
        if action == "dock":
            self.dockPublisher.publish(emptyMessage)
        elif action == "undock":
            self.undockPublisher.publish(emptyMessage)
        else:
            # actionMessage = Twist()  # the mess
            # handle basic movement commands from actions topic
            self.drivePublisher.publish(message)


# Get a Twist message which consists of a linear and angular component which can be negative or positive.
#
# linear.x  (+)     Move forward (m/s)
#           (-)     Move backward (m/s)
#
# angular.z (+)     Rotate counter-clockwise (rad/s)
#          (-)     Rotate clockwise (rad/s)
#
# Limits:
# -0.5 <= linear.x <= 0.5 and -4.25 <= angular.z <= 4.25 (4rads = 45deg)

def angular_speed(current_angle, current_distance, target_distance, time_interval):
    distance_difference = target_distance - current_distance
    angle_difference = math.atan2(distance_difference, current_distance) - current_angle
    angular_speed = angle_difference / time_interval
    return angular_speed


# Main execution
def main():
    rclpy.init()
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)


# Start things up
if __name__ == '__main__':
    main()
