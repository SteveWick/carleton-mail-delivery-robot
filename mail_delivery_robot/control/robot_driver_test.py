#!/usr/bin/env python
import unittest
import robotDriver
import rospy
from std_msgs.msg import String
from time import sleep
import rostest


# This function simulates how messages are sent to the robot driver node
def simulate_message():
    perception_publisher = rospy.Publisher('preceptions', String, 10)
    navigation_map_publisher = rospy.Publisher('navigationMap', String, 10)
    bump_event_publisher = rospy.Publisher('bumpEvent', String, 10)

    # TODO need to extract the exact message received from the beacons topic
    perception_message = ""
    navigation_map_message = ""
    bump_event_message = ""

    perception_publisher.publish(perception_message)
    navigation_map_publisher.publish(navigation_map_message)
    bump_event_publisher.publish(bump_event_message)

    sleep(1)
    del perception_publisher
    del navigation_map_publisher
    del bump_event_publisher


# Test class for testing the throughput of the Robot Driver node
class RobotDriverNodeTest(unittest.TestCase):
    dataTransferSuccess = False

    def callback(self, data):
        # TODO add checking the content of the message processed
        self.dataTransferSuccess = True

    def test_handling_bump_event(self):
        rospy.init_node('test_captain')
        rospy.subscriber('/actions', String, self.callback)
        simulate_message()

        counter = 0
        while not rospy.is_shutdown() and counter < 5 and (not self.dataTransferSuccess):
            sleep(1)
            counter += 1

        self.assertEqual(True, self.dataTransferSuccess)


class DriverStateMachineTest(unittest.TestCase):
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


# class DriverStateTest(unittest.TestCase):
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

# class DockTest(unittest.TestCase):
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


# class VerifyWallTest(unittest.TestCase):
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


# class NoWallTest(unittest.TestCase):
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


# class WallFollowTest(unittest.TestCase):
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


# class RightTurnApproachTest(unittest.TestCase):
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


# class RightTurnArrivedTest(unittest.TestCase):
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


# class HeadOnCollisionInitialTest(unittest.TestCase):
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


# class HeadOnCollisionAvoidedTest(unittest.TestCase):
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


# class GrazeTest(unittest.TestCase):
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


# class PassThroughApproachTest(unittest.TestCase):
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


# class PassThroughArrived(unittest.TestCase):
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


# class LeftTurnApproachTest(unittest.TestCase):
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


# class LeftTurnPassThroughTest(unittest.TestCase):
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

# class LeftTurnRightWallFoundTest(unittest.TestCase):
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

# class LeftTurnLeftWallFoundTest(unittest.TestCase):
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
    rostest.rosrun('control', 'test_robot_driver', RobotDriverNodeTest)
    rostest.rosrun('navigation', 'test_junction_slope_tracker', JunctionSlopeTracker)
