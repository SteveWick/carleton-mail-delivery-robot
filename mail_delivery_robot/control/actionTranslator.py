#!/usr/bin/env python

# @author: Simon Yacoub and Devon Daley (built on top of previous year's work)

# SUBSCRIBER:   String object from 'actions' node
# PUBLISHER:    Twist object to 'cmd_vel' node
#               null object to 'dock' node

import rclpy
from rclpy.node import Node
import re
from std_msgs.msg import String
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist
import time

# This script is meant to take all the action decisions from our reasoner and publish them to the roomba (via cmd_vel)

class ActionTranslator(Node):
    def __init__(self):
        super().__init__('action_translator')
        self.drivePublisher = self.create_publisher(Twist,'cmd_vel',2)
        self.undockPublisher = self.create_publisher(Empty,'dock',1)
        self.dockPublisher = self.create_publisher(Empty,'undock',1)

        self.subscription = self.create_subscription(String,'actions',self.decodeAction,10)


    # Decode and execute the action
    def decodeAction(self, data):
        actionMessage = Twist() #the mess

        # Get the parameters
        action = str(data.data)
        # (drivePublisher, dockPublisher, undockPublisher) = args
        
        #handle basic movement commands from actions topic
        actionMessage = getTwistMesg(action)
        if(action == "left"): #Does a 45 degree turn left (stops robot first)
            actionMessage = getTwistMesg("left")
            tmp = String()
            tmp.data = "stop"
            self.drivePublisher.publish(actionMessage)
            self.get_logger().info("Action: left")

        elif(action == "right"): #Does 45 degree turn right (stops robot first)
            actionMessage = getTwistMesg("right")
            tmp = String()
            tmp.data = "stop"
            self.drivePublisher.publish(actionMessage)
            self.get_logger().info("Action: right")

        # Handle the docking station cases
        if action == "dock":
            self.dockPublisher.publish(Empty())
        elif action == 'undock':
            self.undockPublisher.publish(Empty())
        else:
            #publish action
            self.drivePublisher.publish(actionMessage)
        
        
'''
Get a Twist message which consists of a linear and angular component which can be negative or positive.

linear.x  (+)     Move forward (m/s)
          (-)     Move backward (m/s)

angular.z (+)     Rotate counter-clockwise (rad/s)
         (-)     Rotate clockwise (rad/s)

Limits:
-0.5 <= linear.x <= 0.5 and -4.25 <= angular.z <= 4.25 (4rads = 45deg)
'''
def getTwistMesg(action):
    message = Twist()
    
    if action == "forward":
        message.linear.x = 0.1
        message.angular.z = 0.0
    elif action == "backward":
        message.linear.x = -0.2
        message.linear.z = 0.0
    elif action == "left":
        message.linear.x = 0.0
        message.angular.z = 4.0
    elif action == "right":
        message.linear.x = 0.0
        message.angular.z = -4.0
    elif action == "sleft":
        message.linear.x = 0.05
        message.angular.z = 0.5
    elif action == "sright":
        message.linear.x = 0.05
        message.angular.z = -0.5
    elif action == "avoidright":
        message.linear.x = 0.08
        message.angular.z = -0.5
    elif action == "bleft":
        message.linear.x = -0.1
        message.angular.z = 0.5
    elif action == "stop":
        message.linear.x = 0.0
        message.angular.z = 0.0
    
    return message

# Main execution
def main():
    rclpy.init()
    action_translator = ActionTranslator()
    rclpy.spin(action_translator)

# Start things up
if __name__ == '__main__':
    main()
