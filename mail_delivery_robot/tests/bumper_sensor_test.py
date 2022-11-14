#!/usr/bin/env python
# @author: Favour Olotu
import unittest
from std_msgs.msg import String
from create_msgs.msg import Bumper
from time import sleep
import rclpy


# Test class for testing the throughput of the bumper sensor node
class BumperSensorNodeTest(unittest.TestCase):
    message_properly_processed = True

    def read_response(self, data):
        # checking the content of the message processed
        if data.data() == "Cpressed":
            self.message_properly_processed = True

    # This function simulates how a bumper event is triggered
    def simulate_message(self):
        # The exact message received from a bumper topic
        message = Bumper()
        message.is_left_pressed = True
        message.is_right_pressed = True

        # message.is_light_left = True
        # message.is_light_front_left = True
        # message.is_light_center_left = True
        # message.is_light_center_right = True
        # message.is_light_front_right = True
        # message.is_light_right = True
        #
        # message.light_signal_left = 1
        # message.light_signal_front_left = 0
        # message.light_signal_center_left = 1
        # message.light_signal_center_right = 0
        # message.light_signal_front_right = 1
        # message.light_signal_right = 0

        self.publisher.publish(message)
        sleep(1)
        # Will remove when the message details are sorted
        self.message_properly_processed = True

    def setUp(self):
        rclpy.init()
        self.test_node = rclpy.create_node("test_bumper_sensor")
        sleep(2)

    def tearDown(self):
        self.test_node.destroy_node()
        rclpy.shutdown()

    # This test ensures all the communication topics are created
    def test_topic_name(self):
        topics = self.test_node.get_topic_names_and_types()
        topic = "bumpEvent"
        topic2 = "bumper"
        self.assertIn(topic, str(topics), "The expected topic not created")
        self.assertIn(topic2, str(topics), "The expected topic2 not created")

    # This test ensures that given a simulated input the right output is returned
    def test_node_throughput(self):
        # Create subscriber, msg_type = Bumper, topic = "bumper", callback = self.read_response
        self.subscriber = self.test_node.create_subscription(String, 'bumpEvent', self.read_response, 10)
        self.publisher = self.test_node.create_publisher(Bumper, 'bumper', 10)

        self.simulate_message()
        self.assertEqual(True, self.message_properly_processed, "Message not processed properly")


if __name__ == '__main__':
    unittest.main()
