import time
import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import sys
from bluepy.btle import Scanner, DefaultDelegate
import csv

# Class instantiation which enables handleNotification and handleDiscovery debugging logs
class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)

def loadBeaconData():
    """
    Loads all beacons into a list from beaconList.csv
    :return: List
    """
    beacons = {}
    with open('/var/local/beaconList.csv') as csvfile:
        reader = csv.reader(csvfile,delimiter=",")
        for row in reader:
            beacons[row[0]] = row[1] # row[0] = index, row[1] = id
    return beacons

DEBUG = False
class BeaconReader(Node):
    def __init__(self):
        super().__init__('beacon_reader')
        self.beacons = loadBeaconData()
        for beacon in self.beacons:
            self.get_logger().info(beacon) # Fill logger with each beacon data
        self.publisher_ = self.create_publisher(String, 'beacons', 10) # Create a publisher of String msgs named beacons
        timer_period = 0.5 # Seconds
        self.timer = self.create_timer(timer_period, self.checkForBeacons) # call checkForBeacons() every 0.5 seconds
        self.scanner = Scanner().withDelegate(ScanDelegate()) # Create Scanner Class
 

    def checkForBeacons(self):
        devices = self.scanner.scan(0.4) # Listen for ADV_IND packages for 0.4 seconds
        beaconData = String()

        # For each scanned device check if device address matches beacon in list
        for dev in devices:
            for beacon in self.beacons:
                if(self.beacons[beacon] == dev.addr):
                    # Log successful device detection and signal strength
                    self.get_logger().debug("Device {} ({}), RSSI={} dB".format(dev.addr, dev.addrType, dev.rssi))
                    beaconData.data = beacon + "," + str(dev.rssi) # Configure message from beacon data
                    self.publisher_.publish(beaconData) # Publish Message
                    if True:
                        f = open('captainLog.csv', "a") # log to captainLog.csv file
                        f.write(beaconData.data +"\n")
                        f.close()



def main(): # instantiate everything
    rclpy.init()
    beacon_reader = BeaconReader()

    rclpy.spin(beacon_reader)
    


if __name__ == '__main__':
    main()
