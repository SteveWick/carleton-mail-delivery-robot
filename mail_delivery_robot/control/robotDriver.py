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

    def run(self, distanceFlags, captainRequest):
        return self.currentState.run(distanceFlags)

    def next(self, distanceFlags, captainRequest, bumperState):
        self.currentState = self.currentState.next(distanceFlags, captainRequest, bumperState)


class DriverState:
    counter = 0

    def run(self):
        assert 0, "Must be implemented"

    def next(self, distanceFlags, captainRequest, bumperState):
        assert 0, "Must be implemented"

    def toString(self):
        return ""


class Dock(DriverState):
    def run(self, distanceFlags):
        action = String()
        action.data = "0"
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        # ignore distances
        if captainRequest == "undock":
            return DriverStateMachine.findWall
        else:
            return DriverStateMachine.dock

    def toString(self):
        return "Dock"


class VerifyWall(DriverState):
    def run(self, distanceFlags):
        action = String()
        action.data = "forward"
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (distanceFlags["tooFar"]):
            self.counter = 0
            return DriverStateMachine.noWall
        elif (self.counter > int(magicNumbers['FIND_WALL_TICKS'])):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            self.counter += 1
            return DriverStateMachine.verifyWall

    def toString(self):
        return "VerifyWall"


class NoWall(DriverState):
    def run(self, distanceFlags):
        action = String()
        if self.counter > int(magicNumbers['FIND_WALL_SPIN_TICKS']):
            self.counter = 0
            # left
            action.data = "left"
        else:
            self.counter += 1
            action.data = "0"
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (not distanceFlags["tooFar"] and not distanceFlags["tooTight"] and not distanceFlags["tooWide"]):
            return DriverStateMachine.verifyWall
        else:
            return DriverStateMachine.noWall

    def toString(self):
        return "NoWall"


class WallFollow(DriverState):
    def run(self, distanceFlags):
        action = String()
        if ((distanceFlags["tooFar"] or distanceFlags["wideAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sright"
        elif ((distanceFlags["tooClose"] or distanceFlags["tightAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sleft"
        else:
            action.data = "forward"
            self.counter = 0
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (bumperState == "Cpressed"):
            return DriverStateMachine.headOnCollisionInitial
        if (bumperState == "Rpressed"):
            return DriverStateMachine.graze
        if (captainRequest == "rturn"):
            return DriverStateMachine.rightTurnApproach
        return DriverStateMachine.wallFollow

    def toString(self):
        return "WallFollow"


class RightTurnApproach(DriverState):
    def run(self, distanceFlags):
        action = String()
        if ((distanceFlags["tooFar"] or distanceFlags["wideAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sright"
        elif ((distanceFlags["tooClose"] or distanceFlags["tightAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sleft"
        else:
            action.data = "forward"
            self.counter = 0
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (distanceFlags["tooFar"]):
            return DriverStateMachine.rightTurnArrived
        return DriverStateMachine.rightTurnApproach

    def toString(self):
        return "RightTurnApproach"


class RightTurnArrived(DriverState):
    def run(self, distanceFlags):
        action = String()
        if (self.counter < int(magicNumbers['RIGHT_TURN_PRE_FORWARD_TICKS'])):
            action.data = "forward"
        elif (self.counter < int(magicNumbers['RIGHT_TURN_TICKS']) + int(magicNumbers['RIGHT_TURN_PRE_FORWARD_TICKS'])):
            action.data = "right"
        elif (self.counter < int(magicNumbers['RIGHT_TURN_POST_FORWARD_TICKS']) + int(
                magicNumbers['RIGHT_TURN_PRE_FORWARD_TICKS'])):
            action.data = "forward"
        self.counter += 1
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (self.counter > int(magicNumbers['RIGHT_TURN_POST_FORWARD_TICKS']) + int(
                magicNumbers['RIGHT_TURN_PRE_FORWARD_TICKS'])):
            return DriverStateMachine.wallFollow
        else:
            captainRequest = ""
            return DriverStateMachine.rightTurnArrived

    def toString(self):
        return "rightTurnArrived"


class HeadOnCollisionInitial(DriverState):
    def run(self, distanceFlags):
        action = String()
        if (self.counter < int(magicNumbers['COLLISION_BACK_TICKS'])):
            action.data = "backward"
        elif (self.counter < (magicNumbers['COLLISION_LEFT_TICKS'])):
            action.data = "left"
        else:
            action.data = "creepForward"
        self.counter += 1
        return action

    def next(self, distanceFlags, captainRequest, bumperState):

        if (self.counter > int(magicNumbers['COLLISION_RETURN_MIN_TICKS']) and distanceFlags["tooFar"]):
            self.counter = 0
            return DriverStateMachine.headOnCollisionAvoided
        elif (bumperState == "Rpressed"):
            return DriverStateMachine.graze
        elif (self.counter > int(magicNumbers['COLLISION_WALL_FOLLOW_TICKS'])):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            return DriverStateMachine.headOnCollisionInitial

    def toString(self):
        return "HeadOnCollisionInitial"


class HeadOnCollisionAvoided(DriverState):
    def run(self, distanceFlags):
        action = String()
        if (self.counter < int(magicNumbers['COLLISION_RETURN_RIGHT_TICKS'])):
            action.data = "right"
        elif (self.counter < int(magicNumbers['COLLISION_RETURN_FORWARD_TICKS'])):
            action.data = "forward"
        elif (self.counter % 3 == 0):
            action.data = "sright"
        else:
            action.data = "forward"
        self.counter += 1
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (bumperState == "Rpressed"):
            return DriverStateMachine.graze
        if (self.counter > int(magicNumbers['COLLISION_RETURN_WALL_FOLLOW_TICKS'])):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            return DriverStateMachine.headOnCollisionAvoided

    def toString(self):
        return "HeadOnCollisionAvoided"


class Graze(DriverState):
    def run(self, distanceFlags):
        action = String()
        if (self.counter == 0):
            action.data = "backward"
        if (self.counter < int(magicNumbers['GRAZE_SLEFT_TICKS'])):
            action.data = "sleft"
        self.counter += 1
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (self.counter > int(magicNumbers['GRAZE_WALL_FOLLOW_TICKS'])):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            return DriverStateMachine.graze

    def toString(self):
        return "Graze"


class PassThroughApproach(DriverState):
    def run(self, distanceFlags):
        action = String()
        if ((distanceFlags["tooFar"] or distanceFlags["wideAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sright"
        elif ((distanceFlags["tooClose"] or distanceFlags["tightAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sleft"
        else:
            action.data = "forward"
            self.counter = 0
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (distanceFlags["tooFar"]):
            return DriverStateMachine.passThroughArrived
        return DriverStateMachine.passThroughApproach

    def toString(self):
        return "PassThroughApproach"


class PassThroughArrived(DriverState):
    def run(self, distanceFlags):
        action = String()
        action.data = "forward"
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (distanceFlags["tooFar"]):
            return DriverStateMachine.passThroughArrived
        if (bumperState == "Cpressed"):
            return DriverStateMachine.headOnCollisionInitial
        if (bumperState == "Rpressed"):
            return DriverStateMachine.graze
        return DriverStateMachine.wallFollow

    def toString(self):
        return "PassThroughArrived"


class LeftTurnApproach(DriverState):
    def run(self, distanceFlags):
        action = String()
        if ((distanceFlags["tooFar"] or distanceFlags["wideAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sright"
        elif ((distanceFlags["tooClose"] or distanceFlags["tightAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sleft"
        else:
            action.data = "forward"
            self.counter = 0
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (distanceFlags["tooFar"]):
            return DriverStateMachine.leftTurnPassThrough
        return DriverStateMachine.leftTurnApproach

    def toString(self):
        return "LeftTurnApproach"


class LeftTurnPassThrough(DriverState):
    def run(self, distanceFlags):
        action = String()
        action.data = "forward"
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (distanceFlags["tooFar"]):
            return DriverStateMachine.leftTurnPassThrough
        return DriverStateMachine.leftTurnRightWallFound

    def toString(self):
        return "LeftTurnPassThrough"


class LeftTurnRightWallFound(DriverState):
    def run(self, distanceFlags):
        action = String()
        if (self.counter < int(magicNumbers['LEFT_TURN_TICKS'])):
            action.data = "left"
        else:
            action.data = "forward"
        self.counter += 1
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (bumperState == "Cpressed" or bumperState == "Rpressed" or bumperState == "Lpressed"):
            return DriverStateMachine.leftTurnLeftWallFound
        return DriverStateMachine.leftTurnRightWallFound

    def toString(self):
        return "LeftTurnRightWallFound"


class LeftTurnLeftWallFound(DriverState):
    def run(self, distanceFlags):
        action = String()
        if (self.counter < int(magicNumbers['BACKOFF_TICKS'])):
            action.data = "backward"
        elif (self.counter < int(magicNumbers['RIGHT_TURN_TICKS']) + int(magicNumbers['BACKOFF_TICKS'])):
            action.data = "left"
        else:
            action.data = "forward"
        self.counter += 1
        return action

    def next(self, distanceFlags, captainRequest, bumperState):
        if (self.counter > int(magicNumbers['RIGHT_TURN_TICKS']) + int(magicNumbers['BACKOFF_TICKS'])):
            return DriverStateMachine.rightTurnApproach
        return DriverStateMachine.leftTurnLeftWallFound

    def toString(self):
        return "LeftTurnRightWallFound"


# Initialize states
DriverStateMachine.dock = Dock()
DriverStateMachine.wallFollow = WallFollow()
DriverStateMachine.verifyWall = VerifyWall()
DriverStateMachine.noWall = NoWall()
DriverStateMachine.rightTurnApproach = RightTurnApproach()
DriverStateMachine.rightTurnArrived = RightTurnArrived()
DriverStateMachine.headOnCollisionInitial = HeadOnCollisionInitial()
DriverStateMachine.headOnCollisionAvoided = HeadOnCollisionAvoided()
DriverStateMachine.graze = Graze()
DriverStateMachine.leftTurnApproach = LeftTurnApproach()
DriverStateMachine.leftTurnPassThrough = LeftTurnPassThrough()
DriverStateMachine.leftTurnRightWallFound = LeftTurnRightWallFound()
DriverStateMachine.leftTurnLeftWallFound = LeftTurnLeftWallFound()
DriverStateMachine.passThroughApproach = PassThroughApproach()
DriverStateMachine.passThroughArrived = PassThroughArrived()

DEBUG = False


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
        self.captainRequest = 0
        self.bumperState = "unpressed"
        self.actionPublisher = self.create_publisher(String, 'actions', 2)
        self.IRSubscriber = self.create_subscription(String, 'preceptions', self.updateDistance, 10)
        self.mapSubscriber = self.create_subscription(String, 'navigationMap', self.updateMapState, 10)
        self.bumperEventSubscriber = self.create_subscription(String, 'bumpEvent', self.updateBumperState, 10)
        timer_period = float(magicNumbers['TIMER_PERIOD'])  # Seconds
        self.timer = self.create_timer(timer_period, self.determineAction)
        self.driverStateMachine = DriverStateMachine(DriverStateMachine.rightTurnApproach)

    def determineAction(self):
        action = self.driverStateMachine.run(self.distanceFlags, self.captainRequest)

        if (DEBUG_MODE):
            self.get_logger().info("DriverState: " + self.driverStateMachine.currentState.toString())
            for key in self.distanceFlags:
                self.get_logger().info(str(key) + " : " + str(self.distanceFlags[key]))

        if (action.data != 0):
            self.get_logger().debug("Publishing: " + action.data)
            self.actionPublisher.publish(action)

            if DEBUG:
                f = open('/var/log/mailDeliveryRobot/driverLog.csv', "a")
                f.write(self.driverStateMachine.currentState.toString() + "," + str(self.distance) + "," + str(
                    self.angle) + "," + str(action.data) + "," + str(time.time()) + "\n")
                f.close()

    def updateMapState(self, data):
        self.captainRequest = data.data
        self.driverStateMachine.next(self.distanceFlags, self.captainRequest, self.bumperState)
        self.get_logger().info("Captain: " + self.captainRequest)

    def updateCaptainRequest(self, request):
        # TODO:
        pass

    def updateDistance(self, data):
        if (data.data != "-1"):
            self.distance = data.data.split(",")[0]
            self.angle = data.data.split(",")[1]
            self.distanceFlags["tooFar"] = float(self.distance) > float(magicNumbers['MAX_TARGET_WALL_DISTANCE'])
            self.distanceFlags["tooClose"] = float(self.distance) < float(magicNumbers['MIN_TARGET_WALL_DISTANCE'])
            self.distanceFlags["tightAngle"] = float(self.angle) < float(magicNumbers['MIN_TARGET_WALL_ANGLE'])
            self.distanceFlags["wideAngle"] = float(self.angle) > float(magicNumbers['MAX_TARGET_WALL_ANGLE'])

            self.driverStateMachine.next(self.distanceFlags, self.captainRequest, self.bumperState)
        if (DEBUG_MODE):
            self.get_logger().info("Distance: " + str(self.distance) + "Angle: " + str(self.angle))


    def updateBumperState(self, data):
        self.bumperState = data.data
        self.driverStateMachine.next(self.distanceFlags, self.captainRequest, self.bumperState)
        self.get_logger().debug("Bumper State: " + self.bumperState)


def main():
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)


if __name__ == '__main__':
    main()
