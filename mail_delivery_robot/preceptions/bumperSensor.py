#!/usr/bin/env python

# @author: Simon Yacoub

from std_msgs.msg import String
from create_msgs.msg import Bumper 
import rclpy
from rclpy.node import Node


DEBUG = False
class BumperSensor(Node):
    def __init__(self):
        super().__init__('bumper_sensor')
        self.bumperSubscriber = self.create_subscription(Bumper,'bumper', self.readBump,10)
        self.publisher_ = self.create_publisher(String, 'bumpEvent' , 10)
 

    def readBump(self, data):
        # Extract the publisher and the message data
        is_left_pressed = data.is_left_pressed
        is_right_pressed = data.is_right_pressed 
        
        # Bumper light sensors (Create 2 only) in order from left to right
        # Value = true if an obstacle detected
        is_light_left = data.is_light_left
        is_light_front_left = data.is_light_front_left
        is_light_center_left = data.is_light_center_left
        is_light_center_right = data.is_light_center_right
        is_light_front_right = data.is_light_front_right
        is_light_right = data.is_light_right

        bumpEvent = String()
        if(is_left_pressed and is_right_pressed):bumpEvent.data = "Cpressed"
        elif(is_left_pressed):bumpEvent.data = "Lpressed"
        elif(is_right_pressed):bumpEvent.data = "Rpressed"
        else: bumpEvent.data = "unpressed"

        # if(data.is_light_front_right or  is_light_center_right):
        #     message.data = "bumper detects an object to the left"
        # elif(is_light_center_left or is_light_front_left):
        #     message.data = "bumper detects an object to the right"

        # Publish the perception
        self.get_logger().debug(bumpEvent.data)
        self.publisher_.publish(bumpEvent)


def main():
    rclpy.init()
    bumper_sensor = BumperSensor()

    rclpy.spin(bumper_sensor)
    


if __name__ == '__main__':
    main()



