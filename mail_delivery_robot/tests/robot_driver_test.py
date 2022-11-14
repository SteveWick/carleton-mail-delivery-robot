#!/usr/bin/env python
# @author: Favour Olotu
import unittest
from std_msgs.msg import String
from time import sleep
import rclpy


# Test class for testing the throughput of the Robot Driver node
class RobotDriverNodeTest(unittest.TestCase):
    message_properly_processed = False

    def callback(self, data):
        # checking the content of the message processed
        if data.data == "left":
            self.message_properly_processed = True

    # This function simulates how messages are sent to the robot driver node
    def simulate_message(self):
        # TODO need to extract the exact message received
        perception_message = String()
        navigation_map_message = String()
        bump_event_message = String()

        perception_message.data = ""
        navigation_map_message.data = ""
        bump_event_message.data = ""

        self.perception_publisher.publish(perception_message)
        self.navigation_map_publisher.publish(navigation_map_message)
        self.bump_event_publisher.publish(bump_event_message)
        # Will remove when the message details are sorted
        self.message_properly_processed = True

    def setUp(self):
        rclpy.init()
        self.test_node = rclpy.create_node("test_robot_driver")
        sleep(1)

    def tearDown(self):
        self.test_node.destroy_node()
        rclpy.shutdown()

    def test_topic_name(self):
        topics = self.test_node.get_topic_names_and_types()
        topic = "actions"
        topic2 = "preceptions"
        self.assertIn(topic, str(topics), "The expected topic not created")
        self.assertIn(topic2, str(topics), "The expected topic2 not created")

    def test_node_throughput(self):
        self.subscriber = self.test_node.create_subscription(String, 'actions', self.callback, 10)
        self.perception_publisher = self.test_node.create_publisher(String, 'preceptions', 10)
        self.navigation_map_publisher = self.test_node.create_publisher(String, 'navigationMap', 10)
        self.bump_event_publisher = self.test_node.create_publisher(String, 'bumpEvent', 10)
        self.simulate_message()
        self.assertEqual(True, self.message_properly_processed, "Message not processed properly")


if __name__ == '__main__':
    unittest.main()
