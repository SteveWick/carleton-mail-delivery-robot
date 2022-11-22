#!/usr/bin/env python
#@author: Chase Scott
import unittest
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import sleep
import rclpy
from rclpy.node import Node

# Test class for testing the perceptions Node
# Publishes each of the sensors detected to the perceptions node
# Subscribes to robot_driver to verify that the robot is recieving the correct data from the sensors
class PerceptionsTest(Node):

    test_count = 0

    #TODO these numbers will need tuning also need to be read from csv file
    magicNumbers = {
        'MAX_TARGET_WALL_DISTANCE': 20.0,
        'MIN_TARGET_WALL_DISTANCE': 10.0,
        'MAX_TARGET_WALL_ANGLE': 120.0,
        'MIN_TARGET_WALL_ANGLE': 60.0,
    }

    def __init__(self):
        super().__init__('perceptions_test')

        ##Create publisher and subscriber
        self.action_publisher = self.create_publisher(String, 'preceptions', 2)
        self.subscriber = self.create_subscription(String, 'robot_driver', self.callback, 10)

        self.get_logger().info('Executing tests...')

        ##Repeat test loop every 5 seconds
        timer_time = 5
        timer = self.create_timer(timer_time, self.test_decode_action)


    ##Function called when the subcriber hears a message on the robot_driver topic that preceptions publishes to
    def callback(self, data):
        ##Check if returned values from robotDriver are as expected
        if(data.data != "-1"):
            self.distance = data.data.split(",")[0]
            self.angle = data.data.split(",")[1]
            # these decode the sensor data is in relation to the magic numbers
            print('tooFar = ', float(self.distance) > float(magicNumbers['MAX_TARGET_WALL_DISTANCE']))
            print('tooFar = ', float(self.distance) < float(magicNumbers['MIN_TARGET_WALL_DISTANCE']))
            print('tooFar = ', float(self.angle) < float(magicNumbers['MIN_TARGET_WALL_ANGLE']))
            print('tooFar = ', float(self.angle) > float(magicNumbers['MAX_TARGET_WALL_ANGLE']))

            self.get_logger().info("TEST 1 PASSED")
            ##Increment test count so next loop runs test 2
            self.test_count = 1

        #print the distance and
        self.get_logger().info("Distance: " + str(self.distance) + "Angle: " + str(self.angle))

    #TODO
    def test_decode_action(self):

        data = String()

        #Run test one - Publishes command to robotDriver topic
        if self.test_count == 0:
            self.get_logger().info(f"Sending distance and angle")

            action_message.data = "12.3,78.9"
            self.action_publisher.publish(data)

        #Exit
        self.destroy_node()

#Initialize node
def main(args = None):
        rclpy.init()

        test_node = PerceptionsTest()

        #Run Node on interval specified in the __init__ function, until it is killed
        rclpy.spin(test_node)

        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

