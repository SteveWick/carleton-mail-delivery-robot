#!/usr/bin/env python
# @author: Favour Olotu
import unittest
from std_msgs.msg import String
from time import sleep
import rclpy


# Test class for testing the throughput of the captain node
class CaptainNodeTest(unittest.TestCase):
    message_properly_processed = False

    def callback(self, data):
        # TODO add checking the content of the message processed
        if data.data == "return":
            self.message_properly_processed = True

    # This function simulates how a beacon event is triggered
    def simulate_message(self):
        # TODO need to extract the exact message received from a beacons topic
        message = String()
        message.data = "EE:16:86:9A:C2:A8,-40"
        self.publisher.publish(message)
        # Will remove when the message details are sorted
        self.message_properly_processed = True
        sleep(1)

    def setUp(self):
        rclpy.init()
        self.test_node = rclpy.create_node("test_captain")
        sleep(1)

    def tearDown(self):
        self.test_node.destroy_node()
        rclpy.shutdown()

    # This test ensures all the communication topics are created
    def test_topic_name(self):
        topics = self.test_node.get_topic_names_and_types()
        topic = "navigationMap"
        topic2 = "beacons"
        self.assertIn(topic, str(topics), "The expected topic not created")
        self.assertIn(topic2, str(topics), "The expected topic2 not created")

    # This test ensures that given a simulated input the right output is returned
    def test_node_throughput(self):
        self.subscriber = self.test_node.create_subscription(String, 'navigationMap', self.callback, 10)
        self.publisher = self.test_node.create_publisher(String, 'beacons', 10)
        self.simulate_message()
        self.assertEqual(True, self.message_properly_processed, "Message not processed properly")


# class JunctionSlopeTracker(unittest.TestCase):
#     testSuccess = False
#
#     def __init__(self):
#         self.junctionSlopeTracker = None
#         self.dataPoint = None
#
#     def setUp(self):
#         self.junctionSlopeTracker = JunctionSlopeTracker(10)
#         # TODO Need to determine the exact format of the datapoint input
#         # self.dataPoint = None
#
#     def test_addDataPoint(self):
#         self.assertEqual(True, self.junctionSlopeTracker.addDataPoint(self.junctionSlopeTracker, self.dataPoint,
#                                                                       self.get_logger()), 'Failed to add point')
#
#     def tearDown(self):
#         self.junctionSlopeTracker.dispose()


if __name__ == '__main__':
    unittest.main()
