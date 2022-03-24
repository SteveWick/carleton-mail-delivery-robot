import time
import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import sys
from bluepy.btle import Scanner, DefaultDelegate
import csv


class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

def loadBeaconData():
    beacons = {}
    with open('/var/local/beaconList.csv') as csvfile:
        reader = csv.reader(csvfile,delimiter=",")
        for row in reader:
            beacons[row[0]]= row[1]
    return beacons

DEBUG = False
class BeaconReader(Node):
    def __init__(self):
        super().__init__('beacon_reader')
        self.beacons = loadBeaconData()
        # for beacon in self.beacons:
        #     self.get_logger().info(beacon)
        self.publisher_ = self.create_publisher(String, 'beacons' , 10)
        timer_period = 0.5 #Seconds
        self.timer = self.create_timer(timer_period, self.checkForBeacons)
        self.scanner = Scanner().withDelegate(ScanDelegate())
 

    def checkForBeacons(self):
        devices = self.scanner.scan(0.4)
        beaconData = String()
        for dev in devices:
            for beacon in self.beacons:
                if(self.beacons[beacon] == dev.addr):
                    self.get_logger().info("Device {} ({}), RSSI={} dB".format(dev.addr, dev.addrType, dev.rssi))
                    beaconData.data = beacon + "," + str(dev.rssi)
                    self.publisher_.publish(beaconData)
                    if True:
                        f = open('captainLog.csv', "a")
                        f.write(beaconData.data +"\n")
                        f.close()



def main():
    rclpy.init()
    beacon_reader = BeaconReader()

    rclpy.spin(beacon_reader)
    


if __name__ == '__main__':
    main()
