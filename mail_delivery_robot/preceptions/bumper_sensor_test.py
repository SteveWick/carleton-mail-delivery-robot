#!/usr/bin/env python
import unittest
import rospy
from std_msgs.msg import String
from time import sleep
import rostest


# This function simulates how a bumper event is triggered
def simulate_message():
    publisher = rospy.Publisher('bumper', String, 10)
    # TODO need to extract the exact message received from a bumper topic
    message = ""
    publisher.publish(message)
    sleep(1)
    del publisher


# Test class for testing the throughput of the bumper sensor node
class BumperSensorNodeTest(unittest.TestCase):
    dataTransferSuccess = False

    def callback(self, data):
        # TODO add checking the content of the message processed
        self.dataTransferSuccess = True

    def test_handling_bump_event(self):
        rospy.init_node('test_bumper_sensor')
        rospy.subscriber('/bumpEvent', String, self.callback)
        simulate_message()

        counter = 0
        while not rospy.is_shutdown() and counter < 5 and (not self.dataTransferSuccess):
            sleep(1)
            counter += 1

        self.assertEqual(True, self.dataTransferSuccess)


if __name__ == '__main__':
    rostest.rosrun('preceptions', 'test_bumper_sensor', BumperSensorNodeTest)
