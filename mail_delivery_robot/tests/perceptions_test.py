#!/usr/bin/env python
#@author: Chase Scott
import unittest
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from time import sleep
import rclpy
from rclpy.node import Node


#TODO these numbers will need tuning also need to be read from csv file
magicNumbers = {
    'TIMER_PERIOD': 0.2
}

# Test class for testing the perceptions Node
# Publishes each of the sensors detected to the perceptions node
# Subscribes to robot_driver to verify that the robot is recieving the correct data from the sensors
class PerceptionsTest(Node):

    test_count = 0

    def __init__(self):
        super().__init__('perceptions_test')

        ##Create publisher and subscriber
        self.perceptions_publisher = self.create_publisher(String, 'preceptions', 2)
        self.subscriber = self.create_subscription(String, 'actions', self.callback, 10)

        self.get_logger().info('Executing tests...')

        ##Repeat test loop every 5 seconds
        timer_period = float(magicNumbers['TIMER_PERIOD'])
        timer = self.create_timer(timer_period, self.test_decode_action)


    ##Function called when the subcriber hears a message on the actions topic that preceptions publishes to
    #TODO using the additional sensors to stimulate the other types of messages
    # forward, sleft, sright, right, creepForward, backward
    def callback(self, data):
        ##Check if returned values from robot_driver are as expected
        if (data.data == 'forward'):
            self.get_logger().info('TEST 1 PASSED')
            self.test_count = 1 
        elif (data.data == 'right'):
            self.get_logger().info('TEST 2 PASSED')
            self.test_count = 2 
        self.get_logger().info(data.data)


    ##Test function that publishes the test data for testing
    def test_decode_action(self):

        data = String()

        #Run test one - Publishes command to robotDriver topic
        if self.test_count == 0:
            #self.get_logger().info("Sending distance=12.3cm and angle=78.9degrees")
            data.data = "12.3,78.9"
            self.perceptions_publisher.publish(data)
        elif self.test_count == 1:
            #self.get_logger().info("Sending distance=35.3cm and angle=78.9degrees")
            data.data = "35.3,78.9"
            self.perceptions_publisher.publish(data)
        else:
            self.get_logger().info("TESTS PASSED SUCESSFULLY")
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
