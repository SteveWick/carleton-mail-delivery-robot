#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   preceptions
# PUBLISHER:    actions
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time

class DriverStateMachine:
    def __init__(self, initialState):
        self.currentState = initialState
    def run(self, distance,angle,captainRequest):
        return self.currentState.run(distance,angle)
    def next(self,distance,angle,captainRequest):
        self.currentState = self.currentState.next(distance,angle,captainRequest)

class DriverState:
    counter = 0
    def run(self):
        assert 0, "Must be implemented"
    def next(self, distance,angle, captainRequest):
        assert 0, "Must be implemented"
    def toString(self):
        return ""

class Dock(DriverState):
    def run(self,distance,angle):
        action = String()
        action.data = "0"
        return action 
    def next(self,distance,angle,captainRequest):
        #ignore distances
        if captainRequest == "undock":
            return DriverStateMachine.findWall
        else:
            return DriverStateMachine.dock
    def toString(self):
        return "Dock"

class FindWall(DriverState):
    def run(self,distance,angle):
        action = String()
        action.data = "forward"
        return action
    def next(self,distance,angle,captainRequest):
        if(float(distance) > 20.0):
            self.counter = 0
            return DriverStateMachine.findWallSpin
        elif(self.counter > 15):
            self.counter = 0
            return DriverStateMachine.wallFollow
        else:
            self.counter += 1
            return DriverStateMachine.findWall
    def toString(self):
        return "FindWall"

class FindWallSpin(DriverState):
    def run(self,distance,angle):
        action = String()
        if self.counter > 5:
            self.counter = 0
            #left
            action.data = "left"
        else:
            self.counter += 1
            action.data = "0"
        return action
    def next(self,distance,angle,captainRequest):
        if(float(distance) < 15.0 and float(angle) > 60.0 and float(angle) < 120.0):
            return DriverStateMachine.findWall
        else:
            return DriverStateMachine.findWallSpin
    def toString(self):
        return "FindWallSpin"

class WallFollow(DriverState):
    def run(self,distance,angle):
        action = String()
        if((float(distance) > 20.0 or float(angle) > 120.0) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sright"
        elif((float(distance) < 10.0 or float(angle) < 60.0) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sleft"
        else:    
            action.data = "forward"
            self.counter = 0
        return action 
    def next(self,distance,angle,captainRequest):
        if(captainRequest == "rturn"):
            return DriverStateMachine.rightTurnApproach
        return DriverStateMachine.wallFollow
    def toString(self):
        return "WallFollow"
    
class RightTurnApproach(DriverState):
    def run(self,distance,angle):
        action = String()
        if((float(distance) > 20.0 or float(angle) > 120.0) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sright"
        elif((float(distance) < 10.0 or float(angle) < 60.0) and self.counter % 5 == 0):
            self.counter += 1
            action.data = "sleft"
        else:    
            action.data = "forward"
            self.counter = 0
        return action 
    def next(self,distance,angle,captainRequest):
        if((float(distance) > 20.0)):
            return DriverStateMachine.rightTurn
        return DriverStateMachine.wallFollow
    def toString(self):
        return "RightTurnApproach"

class RightTurn(DriverState):
    def run(self,distance,angle):
        action = String()
        
        if(self.counter < 3):
            action.data = "right"
        elif(self.counter < 5):
            action.data = "forward"
        self.counter += 1
        return action 
    def next(self,distance,angle,captainRequest):
        if(self.counter > 5):
            return DriverStateMachine.wallFollow
        else:
            captainRequest = ""
            return DriverStateMachine.rightTurn
        
    def toString(self):
        return "RightTurn"

#Initialize states
DriverStateMachine.dock = Dock()
DriverStateMachine.wallFollow = WallFollow()
DriverStateMachine.findWall = FindWall()
DriverStateMachine.findWallSpin = FindWallSpin()
DriverStateMachine.rightTurnApproach = RightTurnApproach()
DriverStateMachine.rightTurn = RightTurn()

DEBUG = False
class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.distance = 0.0
        self.angle = 0.0
        self.captainRequest = 0
        self.actionPublisher = self.create_publisher(String,'actions',2)
        self.IRSubscriber = self.create_subscription(String,'preceptions', self.updateDistance,10)
        self.mapSubscriber = self.create_subscription(String,'navigationMap', self.updateMapState,10)
        timer_period = 0.2 #Seconds
        self.timer = self.create_timer(timer_period, self.determineAction)
        self.driverStateMachine = DriverStateMachine(DriverStateMachine.findWall)


    def determineAction(self):
        action = self.driverStateMachine.run(self.distance,self.angle,self.captainRequest)
        self.get_logger().debug("DriverState: " + self.driverStateMachine.currentState.toString())
        self.get_logger().debug("Distance: " + str(self.distance))

        if(action.data != 0):
            self.get_logger().debug("Publishing: " + action.data)
            self.actionPublisher.publish(action)
            
            if DEBUG:
                f = open('/var/log/mailDeliveryRobot/driverLog.csv', "a")
                f.write(self.driverStateMachine.currentState.toString()+","+str(self.distance)+","+str(self.angle)+","+str(action.data)+","+ str(time.time()) + "\n")
                f.close()
        
    def updateMapState(self, data):
        self.captainRequest = data.data
        self.driverStateMachine.next(self.distance,self.angle,self.captainRequest)
        self.get_logger().info("Captain: " + self.captainRequest)


    def updateCaptainRequest(self,request):
        #TODO:
        pass

    def updateDistance(self, data):
        if(data.data != "-1"):
            self.distance = data.data.split(",")[0]
            self.angle = data.data.split(",")[1]
            self.driverStateMachine.next(self.distance,self.angle,self.captainRequest)
        # self.get_logger().info("Distance: " + str(self.distance) + "Angle: " + str(self.angle))


def main():
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)


if __name__ == '__main__':
    main()