#!/usr/bin/env python
# @author: Stephen Wicklund, Jacob Charpentier
# SUBSCRIBER:   preceptions
# PUBLISHER:    actions
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import csv
import os

# ~~~~ DEBUG MODE ~~~~
DEBUG_MODE = False

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


class DriverStateMachine:
    def __init__(self, initialState):
        self.currentState = initialState

    def handleNewDistanceEvent(self, feedback_value, actionPublisher):
        self.currentState = self.currentState.handleNewDistanceEvent(feedback_value, actionPublisher)

class DriverState:
    def handleNewDistanceEvent(self, feedback_value, actionPublisher):
        assert 0, "Must be implemented"

    def toString(self):
        return ""

class FindWall(DriverState):
    def handleNewDistanceEvent(self, feedback_value, actionPublisher):
        #found wall so change state
        #TODO this should check that a wall is actually found before switching state.
        return WallFollowing()

    def toString(self):
        return "FindWall"

class WallFollowing(DriverState):
    def handleNewDistanceEvent(self, feedback_value, actionPublisher):
        action = String()
        action.data = feedback_value
        actionPublisher.publish(action)
        return self

    def toString(self):
        return "WallFollowing"

class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        # Used for DEBUG only
        self.distance = 0.0
        self.angle = 0.0

        # configure publisher and subscribers
        self.actionPublisher = self.create_publisher(String, 'actions', 2)
        self.IRSubscriber = self.create_subscription(String, 'preceptions', self.updateIRSensor, 10)
        self.bumperEventSubscriber = self.create_subscription(String, 'bumpEvent', self.updateBumperState, 10)
        # TODO These will be implemented in future commits
        # self.mapSubscriber = self.create_subscription(String, 'navigationMap', self.updateMapState, 10)

        # initialize first state
        self.driverStateMachine = DriverStateMachine(FindWall())

    # update the robots distance flags based on data recieved from the IR sensors
    def updateIRSensor(self, data):

        if (data.data != "-1"):
            #self.distance = data.data.split(",")[0]
            #self.angle = data.data.split(",")[1]

            feedback_value = data.data
            #Make sure to pass it through the decode function first. actionTranslater will have to change in order to be able to handle this new message.

            self.driverStateMachine.handleNewDistanceEvent(feedback_value, self.actionPublisher)

        if (DEBUG_MODE):
            self.get_logger().info("Distance: " + str(self.distance) + "Angle: " + str(self.angle))
    
    def updateBumperState(self, data):
        self.bumperState = data.data
        #self.driverStateMachine.next(self.distanceFlags, self.bumperState)
        if (DEBUG_MODE):
            self.get_logger().debug("Bumper State: " + self.bumperState)

def main():
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)

if __name__ == '__main__':
    main()
