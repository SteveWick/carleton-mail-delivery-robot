#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   
# PUBLISHER:    navigationMap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Captain(Node):
    def __init__(self):
        super().__init__('captain')
        self.mapPublisher = self.create_publisher(String,'navigationMap', 10)


def main():
    rclpy.init()
    captain = Captain()
    # rclpy.spin(captain)

    while(True):
        mapUpdate = String()
        mapUpdate.data = input("Enter a navigational update: ")
        captain.mapPublisher.publish(mapUpdate)
        captain.get_logger().debug('Publishing: "%s"' % mapUpdate.data)



if __name__ == '__main__':
    main()