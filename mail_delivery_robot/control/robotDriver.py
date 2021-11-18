#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   preceptions
# PUBLISHER:    actions
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class RobotDriver(Node):
    def __init__(self):
        super().__init__('robot_driver')
        self.driveState = "dock"
        self.lastAction = self.driveState
        self.distance = 0.0
        self.actionPublisher = self.create_publisher(String,'actions',2)
        self.subscriber = self.create_subscription(String,'preceptions', self.updateDistance,10)
        self.subscriber = self.create_subscription(String,'navigationMap', self.updateMapState,10)
        timer_period = 0.1 #Seconds
        self.timer = self.create_timer(timer_period, self.determineAction)

    def determineAction(self):
        action = String()
        if self.driveState == "dock":
            action.data = "dock"
        elif self.driveState == "undock":
            action.data = "undock"
            self.driveState = "straightForward"
        elif self.driveState == "straightForward":
            action.data = "forward"
        elif self.driveState == "tooCloseForward":
            action.data = "slightLeft"
        elif self.driveState == "tooFarForward":
            action.data = "slightRight"
        elif self.driveState == "stop":
            action.data = "stop"
        
        self.actionPublisher.publish(action)
    
    def updateMapState(self, data):
        self.driveState = data.data


    def updateDistance(self, data):
        self.distance = data.data
        if(self.distance > 25):
            self.driveState = "tooFarForward"
        elif(self.distance > 10):
            sekf.driveState = "tooCloseForward"


def main():
    rclpy.init()
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)


if __name__ == '__main__':
    main()