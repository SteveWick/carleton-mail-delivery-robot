#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   beacons
# PUBLISHER:    navigationMap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
class Captain(Node):
    def __init__(self):
        super().__init__('captain')
        self.mapPublisher = self.create_publisher(String,'navigationMap', 10)
        self.beaconSubscriber = self.create_subscription(String,'beacons', self.readBeacon,10)

    def readBeacon(self, beacon):
        self.get_logger().info('Received: "%s"' % beacon.data)
        if False:
            f = open('/var/log/mailDeliveryRobot/captainLog.csv', "a")
            f.write(beacon.data +"\n")
            f.close()

DEBUG = False
def main():
    rclpy.init()
    captain = Captain()
    # rclpy.spin(captain)

    while(DEBUG):
        mapUpdate = String()
        mapUpdate.data = input("Enter a navigational update: ")
        captain.mapPublisher.publish(mapUpdate)
        captain.get_logger().debug('Publishing: "%s"' % mapUpdate.data)
    
    rclpy.spin(captain)



if __name__ == '__main__':
    main()