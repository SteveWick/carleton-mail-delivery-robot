#!/usr/bin/env python
import unittest
import rospy
from std_msgs.msg import String
from time import sleep
import rostest
from captain.py import JunctionSlopeTracker


# This function simulates how a beacon event is triggered
def simulate_message():
    publisher = rospy.Publisher('beacons', String, 10)
    # TODO need to extract the exact message received from the beacons topic
    message = ""
    publisher.publish(message)
    sleep(1)
    del publisher


# Test class for testing the throughput of the captain node
class CaptainNodeTest(unittest.TestCase):
    dataTransferSuccess = False

    def callback(self, data):
        # TODO add checking the content of the message processed
        self.dataTransferSuccess = True

    def test_handling_bump_event(self):
        rospy.init_node('test_captain')
        rospy.subscriber('/navigationMap', String, self.callback)
        simulate_message()

        counter = 0
        while not rospy.is_shutdown() and counter < 5 and (not self.dataTransferSuccess):
            sleep(1)
            counter += 1

        self.assertEqual(True, self.dataTransferSuccess)


class JunctionSlopeTracker(unittest.TestCase):
    testSuccess = False

    def __init__(self):
        self.junctionSlopeTracker = None
        self.dataPoint = None

    def setUp(self):
        self.junctionSlopeTracker = JunctionSlopeTracker(10)
        # TODO Need to determine the exact format of the datapoint input
        # self.dataPoint = None

    def test_addDataPoint(self):
        self.assertEqual(True, self.junctionSlopeTracker.addDataPoint(self.junctionSlopeTracker, self.dataPoint,
                                                                      self.get_logger()), 'Failed to add point')

    def tearDown(self):
        self.junctionSlopeTracker.dispose()


if __name__ == '__main__':
    rostest.rosrun('navigation', 'test_captain', CaptainNodeTest)
    rostest.rosrun('navigation', 'test_junction_slope_tracker', JunctionSlopeTracker)
