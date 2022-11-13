#!/usr/bin/env python
import unittest
from std_msgs.msg import String
from create_msgs.msg import Bumper
from time import sleep
import rclpy


# Test class for testing the throughput of the bumper sensor node
class BumperSensorNodeTest(unittest.TestCase):
    message_properly_processed = False

    def callback(self):
        # TODO add checking the content of the message processed
        self.message_properly_processed = True

    # This function simulates how a bumper event is triggered
    def simulate_message(self):
        # Publisher that sends String messages named bumpEvent
        # self.publisher = self.test_node.create_publisher(String, 'bumper', 10)
        # publisher = rospy.Publisher('bumper', String, 10)
        # TODO need to extract the exact message received from a bumper topic
        message = Bumper()
        # message.data = ""
        self.publisher.publish(message)


    def setUp(self):
        rclpy.init()
        self.test_node = rclpy.create_node("test_bumper_sensor")
        self.subscriber = self.test_node.create_subscription(String, 'bumpEvent', self.callback, 10)
        self.publisher = self.test_node.create_publisher(Bumper, 'bumper', 10)
        sleep(2)
    def tearDown(self):
        self.test_node.destroy_node()
        rclpy.shutdown()

    def test_topic_name(self):
        topics = self.test_node.get_topic_names_and_types()
        topic = "bumpEvent"
        topic2 = "bumper"
        self.assertIn(topic, str(topics), "The expected topic not created")
        self.assertIn(topic2, str(topics), "The expected topic2 not created")

    def test_node_throughput(self):
        # Create subscriber, msg_type = Bumper, topic = "bumper", callback = self.callback
        # self.subscriber = self.test_node.create_subscription(String, 'bumpEvent', self.callback, 10)
        # self.publisher = self.test_node.create_publisher(String, 'bumper', 10)
        # rclpy.spin(self.test_node)
        self.simulate_message()
        self.assertEqual(True, self.message_properly_processed, "Message not processed properly")


if __name__ == '__main__':
    unittest.main()
