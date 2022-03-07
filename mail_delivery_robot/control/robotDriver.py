#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   preceptions
# PUBLISHER:    actions
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import csv

# ~~~~ DEFAULTS ~~~~~
magicNumbers = {
    'MAX_TARGET_WALL_DISTANCE': 20.0,
    'MIN_TARGET_WALL_DISTANCE': 10.0,
    'MAX_TARGET_WALL_ANGLE': 120.0,
    'MIN_TARGET_WALL_ANGLE': 60.0,
    'FIND_WALL_TICKS': 15,
    'FIND_WALL_SPIN_TICKS': 5,
    'RIGHT_TURN_TICKS': 3,
    'RIGHT_TURN_FORWARD_TICKS': 5,
    'COLLISION_BACK_TICKS': 2,
    'COLLISION_LEFT_TICKS': 5,
    'COLLISION_RETURN_MIN_TICKS': 6,
    'COLLISION_WALL_FOLLOW_TICKS': 40,
    'COLLISION_RETURN_RIGHT_TICKS': 2,
    'COLLISION_RETURN_FORWARD_TICKS': 4,
    'COLLISION_RETURN_WALL_FOLLOW_TICKS': 30,
    'GRAZE_SLEFT_TICKS': 3,
    'GRAZE_WALL_FOLLOW_TICKS': 3,
    'TIMER_PERIOD': 0.2,
}

# ~~~~ Load overrides ~~~~
def loadNumberOverrides():
    with open('/var/local/magicNumbers.csv') as csvfile:
        reader = csv.reader(csvfile,delimiter=",")
        for row in reader:
            magicNumbers[row[0]]= row[1]
    return magicNumbers

class DriverStateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
    def run(self, distanceFlags,captainRequest):
        return self.currentState.run(distanceFlags)
    def next(self,distanceFlags,captainRequest,bumperState):
        self.currentState = self.currentState.next(distanceFlags,captainRequest,bumperState)

class DriverState:
    counter = 0
    def run(self):
        assert 0, "Must be implemented"
    def next(self, distanceFlags, captainRequest,bumperState):
        assert 0, "Must be implemented"
    def toString(self):
        return ""

class Dock(DriverState):
    def run(self,distanceFlags):
        action = String()
        action.data = "0"
        return action 
    def next(self,distanceFlags,captainRequest,bumperState):
        #ignore distances
        if captainRequest == "undock":
            return DriverStateMachine.findWall
        else:
            return DriverStateMachine.dock
    def toString(self):
        return "Dock"

class FindWall(DriverState):
    def run(self,distanceFlags):
        action = String()
        action.data = "forward"
        return action
    def next(self,distanceFlags,captainRequest,bumperState):
        if(distanceFlags["tooFar"]):
            self.counter = 0
            return DriverStateMachine.findWallSpin
        elif(self.counter > magicNumbers['FIND_WALL_TICKS']):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            self.counter += 1
            return DriverStateMachine.findWall
    def toString(self):
        return "FindWall"

class FindWallSpin(DriverState):
    def run(self,distanceFlags):
        action = String()
        if self.counter > magicNumbers['FIND_WALL_SPIN_TICKS']:
            self.counter = 0
            #left
            action.data = "left"
        else:
            self.counter += 1
            action.data = "0"
        return action
    def next(self,distanceFlags,captainRequest,bumperState):
        if(not distanceFlags["tooFar"] and not distanceFlags["tooTight"] and not distanceFlags["tooWide"]):
            return DriverStateMachine.findWall
        else:
            return DriverStateMachine.findWallSpin
    def toString(self):
        return "FindWallSpin"

class WallFollow(DriverState):
    def run(self,distanceFlags):
        action = String()
        if((distanceFlags["tooFar"] or distanceFlags["wideAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sright"
        elif((distanceFlags["tooClose"] or distanceFlags["tightAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sleft"
        else:    
            action.data = "forward"
            self.counter = 0
        return action 
    def next(self,distanceFlags,captainRequest, bumperState):
        if(bumperState == "Cpressed"):
            return DriverStateMachine.headOnCollisionAvoid
        if(bumperState == "Rpressed"):
            return DriverStateMachine.graze
        if(captainRequest == "rturn"):
            return DriverStateMachine.rightTurnApproach
        return DriverStateMachine.wallFollow
    def toString(self):
        return "WallFollow"
    
class RightTurnApproach(DriverState):
    def run(self,distanceFlags):
        action = String()
        if((distanceFlags["tooFar"] or distanceFlags["wideAngle"] ) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sright"
        elif((distanceFlags["tooClose"] or distanceFlags["tightAngle"]) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sleft"
        else:    
            action.data = "forward"
            self.counter = 0
        return action 
    def next(self,distanceFlags,captainRequest, bumperState):
        if(distanceFlags["tooFar"]):
            return DriverStateMachine.rightTurn
        return DriverStateMachine.wallFollow
    def toString(self):
        return "RightTurnApproach"

class RightTurn(DriverState):
    def run(self,distanceFlags):
        action = String()
        
        if(self.counter < magicNumbers['RIGHT_TURN_TICKS']):
            action.data = "right"
        elif(self.counter < magicNumbers['RIGHT_TURN_FORWARD_TICKS']):
            action.data = "forward"
        self.counter += 1
        return action 
    def next(self,distanceFlags,captainRequest,bumperState):
        if(self.counter > magicNumbers['RIGHT_TURN_FORWARD_TICKS']):
            return DriverStateMachine.wallFollow
        else:
            captainRequest = ""
            return DriverStateMachine.rightTurn
        
    def toString(self):
        return "RightTurn"

class HeadOnCollisionAvoid(DriverState):
    def run(self,distanceFlags):
        action = String()
        if(self.counter < magicNumbers['COLLISION_BACK_TICKS']):
            action.data = "backward"
        elif(self.counter < magicNumbers['COLLISION_LEFT_TICKS']):
            action.data = "left"
        else:
            action.data = "creepForward"
        self.counter += 1
        return action 
    def next(self,distanceFlags,captainRequest,bumperState):
        
        if(self.counter > magicNumbers['COLLISION_RETURN_MIN_TICKS'] and distanceFlags["tooFar"]):
            self.counter = 0
            return DriverStateMachine.headOnCollisionReturn
        elif(bumperState == "Rpressed"):
            return DriverStateMachine.graze
        elif(self.counter > magicNumbers['COLLISION_WALL_FOLLOW_TICKS']):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            return DriverStateMachine.headOnCollisionAvoid
        
    def toString(self):
        return "HeadOnCollisionAvoid"

class HeadOnCollisionReturn(DriverState):
    def run(self,distanceFlags):
        action = String()
        if(self.counter < magicNumbers['COLLISION_RETURN_RIGHT_TICKS']):
            action.data = "right"
        elif(self.counter < magicNumbers['COLLISION_RETURN_FORWARD_TICKS']):
            action.data = "forward"
        elif(self.counter % 3 == 0):
            action.data = "sright"
        else:
            action.data = "forward"
        self.counter += 1
        return action 
    def next(self,distanceFlags,captainRequest,bumperState):
        if(bumperState == "Rpressed"):
            return DriverStateMachine.graze
        if(self.counter > magicNumbers['COLLISION_RETURN_WALL_FOLLOW_TICKS']):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            return DriverStateMachine.headOnCollisionReturn
        
    def toString(self):
        return "HeadOnCollisionReturn"

class Graze(DriverState):
    def run(self,distanceFlags):
        action = String()
        if(self.counter == 0):
            action.data = "backwards"
        if(self.counter < magicNumbers['GRAZE_SLEFT_TICKS']):
            action.data = "sleft"
        self.counter += 1
        return action 
    def next(self,distanceFlags,captainRequest,bumperState):
        if(self.counter > magicNumbers['GRAZE_WALL_FOLLOW_TICKS']):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            return DriverStateMachine.graze
        
    def toString(self):
        return "Graze"

#Initialize states
DriverStateMachine.dock = Dock()
DriverStateMachine.wallFollow = WallFollow()
DriverStateMachine.findWall = FindWall()
DriverStateMachine.findWallSpin = FindWallSpin()
DriverStateMachine.rightTurnApproach = RightTurnApproach()
DriverStateMachine.rightTurn = RightTurn()
DriverStateMachine.headOnCollisionAvoid = HeadOnCollisionAvoid()
DriverStateMachine.headOnCollisionReturn = HeadOnCollisionReturn()
DriverStateMachine.graze = Graze()

DEBUG = False
class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        #Used for DEBUG only
        self.distance = 0.0
        self.angle = 0.0
        
        #Events
        self.distanceFlags = {
            "tooFar": False,
            "tooClose": False,
            "tightAngle": False,
            "wideAngle": False   
        }
        self.captainRequest = 0
        self.bumperState = "unpressed"
        self.actionPublisher = self.create_publisher(String,'actions',2)
        self.IRSubscriber = self.create_subscription(String,'preceptions', self.updateDistance,10)
        self.mapSubscriber = self.create_subscription(String,'navigationMap', self.updateMapState,10)
        self.bumperEventSubscriber = self.create_subscription(String,'bumpEvent',self.updateBumperState,10)
        timer_period = magicNumbers['TIMER_PERIOD'] #Seconds
        self.timer = self.create_timer(timer_period, self.determineAction)
        self.driverStateMachine = DriverStateMachine(DriverStateMachine.wallFollow)


    def determineAction(self):
        action = self.driverStateMachine.run(self.distanceFlags,self.captainRequest)
        self.get_logger().info("DriverState: " + self.driverStateMachine.currentState.toString())

        if(action.data != 0):
            self.get_logger().debug("Publishing: " + action.data)
            self.actionPublisher.publish(action)
            
            if DEBUG:
                f = open('/var/log/mailDeliveryRobot/driverLog.csv', "a")
                f.write(self.driverStateMachine.currentState.toString()+","+str(self.distance)+","+str(self.angle)+","+str(action.data)+","+ str(time.time()) + "\n")
                f.close()
        
    def updateMapState(self, data):
        self.captainRequest = data.data
        self.driverStateMachine.next(self.distanceFlags,self.captainRequest,self.bumperState)
        self.get_logger().info("Captain: " + self.captainRequest)


    def updateCaptainRequest(self,request):
        #TODO:
        pass

    def updateDistance(self, data):
        if(data.data != "-1"):
            self.distance = data.data.split(",")[0]
            self.angle = data.data.split(",")[1]
            self.distanceFlags["tooFar"] = float(self.distance) > magicNumbers['MAX_TARGET_WALL_DISTANCE']
            self.distanceFlags["tooClose"] = float(self.distance) < magicNumbers['MIN_TARGET_WALL_DISTANCE']
            self.distanceFlags["tightAngle"] = float(self.angle) > magicNumbers['MIN_TARGET_WALL_ANGLE']
            self.distanceFlags["wideAngle"] = float(self.angle) < magicNumbers['MAX_TARGET_WALL_ANGLE']

            self.driverStateMachine.next(self.distanceFlags,self.captainRequest,self.bumperState)
        # self.get_logger().info("Distance: " + str(self.distance) + "Angle: " + str(self.angle))
    
    def updateBumperState(self,data):
        self.bumperState = data.data
        self.driverStateMachine.next(self.distance,self.angle,self.captainRequest,self.bumperState)
        self.get_logger().debug("Bumper State: " + self.bumperState)




def main():
    try:
        loadNumberOverrides()
    except:
        print("No tuning file found!")
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)


if __name__ == '__main__':
    main()