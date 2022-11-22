#!/usr/bin/env python

# @author: Simon Yacoub, Jacob Charpentier

from std_msgs.msg import String
from create_msgs.msg import Bumper
import rclpy
from rclpy.node import Node
import csv
import os

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


DEBUG = False


class BumperSensor(Node):
    def __init__(self):
        super().__init__('bumper_sensor')
        self.counter = 0
        self.lastState = ""
        # Create subscriber, msg_type = Bumper, topic = "bumper", callback = self.readBump
        self.bumperSubscriber = self.create_subscription(Bumper, 'bumper', self.readBump, 10)

        # Publisher that sends String messages named bumpEvent
        self.publisher_ = self.create_publisher(String, 'bumpEvent', 10)

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
        if (is_left_pressed and is_right_pressed):
            bumpEvent.data = "Cpressed"
        elif (is_left_pressed):
            bumpEvent.data = "Lpressed"
        elif (is_right_pressed):
            bumpEvent.data = "Rpressed"
        else:
            bumpEvent.data = "unpressed"

        # if(data.is_light_front_right or  is_light_center_right):
        #     message.data = "bumper detects an object to the left"
        # elif(is_light_center_left or is_light_front_left):
        #     message.data = "bumper detects an object to the right"

        # Publish the perception
        if (self.lastState != bumpEvent.data or self.counter > int(magicNumbers['MAX_BUMP_EVENT_PUBLISH_TICKS'])):
            self.lastState = bumpEvent.data
            self.get_logger().debug(bumpEvent.data)
            self.publisher_.publish(bumpEvent)
            self.counter = 0
        self.counter += 1


def main():
    rclpy.init()
    bumper_sensor = BumperSensor()

    rclpy.spin(bumper_sensor)


if __name__ == '__main__':
    main()
