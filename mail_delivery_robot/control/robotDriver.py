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

    def run(self, distanceFlags):
        return self.currentState.run(distanceFlags)

    def next(self, distanceFlags, bumperState):
        self.currentState = self.currentState.next(distanceFlags, captainRequest, bumperState)


class DriverState:
    counter = 0

    def run(self):
        assert 0, "Must be implemented"

    def next(self, distanceFlags, bumperState):
        assert 0, "Must be implemented"

    def toString(self):
        return ""

# Assigned to Jake
class Docked(DriverState):
    def run(self):
        action = String()
        return action

    def next(self, distanceFlags, bumperState):
        pass

    def toString(self):
        return "Docked"

# Assigned to Jake
class FindWall(DriverState):
    def run(self):
        action = String()
        return action

    def next(self, distanceFlags, bumperState):
        pass

    def toString(self):
        return "FindWall"

# Assigned to Chase
class WallFollowing(DriverState):
    def run(self):
        action = String()
        return action

    def next(self, distanceFlags, bumperState):
        pass

    def toString(self):
        return "WallFollowing"
# Assigned to Chase
class IntersectionHandling(DriverState):
    def run(self):
        action = String()
        return action

    def next(self, distanceFlags, bumperState):
        pass

    def toString(self):
        return "IntersectionHandling"
# Assigned to Chase
class DestinationReached(DriverState):
    def run(self):
        action = String()
        return action

    def next(self, distanceFlags, bumperState):
        pass

    def toString(self):
        return "DestinationReached"
# Assigned to Chase
class CollisionHandling(DriverState):
    def run(self):
        action = String()
        return action
    def next(self, distanceFlags, bumperState):
        pass

    def toString(self):
        return "CollisionHandling"

# Initialize states
DriverStateMachine.Docked = Docked()
DriverStateMachine.FindWall = FindWall()
DriverStateMachine.WallFollowing = WallFollowing()
DriverStateMachine.IntersectionHandling = IntersectionHandling()
DriverStateMachine.DestinationReached = DestinationReached()
DriverStateMachine.CollisionHandling = CollisionHandling()


class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        # Used for DEBUG only
        self.distance = 0.0
        self.angle = 0.0

        # Events
        self.distanceFlags = {
            "tooFar": False,
            "tooClose": False,
            "tightAngle": False,
            "wideAngle": False
        }

        # configure publisher and subscribers
        self.actionPublisher = self.create_publisher(String, 'actions', 2)
        self.IRSubscriber = self.create_subscription(String, 'preceptions', self.updateDistance, 10)
        self.bumperEventSubscriber = self.create_subscription(String, 'bumpEvent', self.updateBumperState, 10)
        # TODO These will be implemented in future commits
        # self.mapSubscriber = self.create_subscription(String, 'navigationMap', self.updateMapState, 10)


        timer_period = float(magicNumbers['TIMER_PERIOD'])  # Seconds
        self.timer = self.create_timer(timer_period, self.determineAction)

        # initialize first state TODO
        self.driverStateMachine = DriverStateMachine()

    def determineAction(self):
        # get current action based on current state's run command
        action = self.driverStateMachine.run(self.distanceFlags)
        # if action doesn't equa 0, publish it to actions
        if (action.data != 0):
            self.get_logger().debug("Publishing: " + action.data)
            self.actionPublisher.publish(action)

    # update the robots distance flags based on data recieved from the IR sensors
    def updateDistance(self, data):
        if (data.data != "-1"):
            self.distance = data.data.split(",")[0]
            self.angle = data.data.split(",")[1]
            self.distanceFlags["tooFar"] = float(self.distance) > float(magicNumbers['MAX_TARGET_WALL_DISTANCE'])
            self.distanceFlags["tooClose"] = float(self.distance) < float(magicNumbers['MIN_TARGET_WALL_DISTANCE'])
            self.distanceFlags["tightAngle"] = float(self.angle) < float(magicNumbers['MIN_TARGET_WALL_ANGLE'])
            self.distanceFlags["wideAngle"] = float(self.angle) > float(magicNumbers['MAX_TARGET_WALL_ANGLE'])

        if (DEBUG_MODE):
            self.get_logger().info("Distance: " + str(self.distance) + "Angle: " + str(self.angle))


def main():
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)


if __name__ == '__main__':
    main()
