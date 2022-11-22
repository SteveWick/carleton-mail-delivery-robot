#!/usr/bin/env python
# @author: Stephen Wicklund, Jacob Charpentier
# SUBSCRIBER:   beacons
# PUBLISHER:    navigationMap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv
import os


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


def sign(x):
    return bool(x > 0) - bool(x < 0)


class JunctionSlopeTracker():
    def __init__(self, N):
        self.dataQueue = []
        self.averageQueue = []
        self.slopeQueue = []
        self.N = N
        self.sum = 0

        self.counter = 0
        self.signalSent = False

    def addDataPoint(self, dataPoint, logger):
        if (len(self.averageQueue) != 0 and abs(int(dataPoint) - int(self.averageQueue[-1])) > int(
                magicNumbers['BEACON_OUTLIER_THRESHOLD'])):
            return False

        # Remove data point exceeding window size N
        if len(self.dataQueue) >= self.N:
            self.sum -= int(self.dataQueue.pop(0))

        # Add new data point and update sum and average queue
        self.dataQueue.append(dataPoint)
        self.sum += int(dataPoint)
        self.averageQueue.append(self.sum / len(self.dataQueue))
        self.counter += 1
        logger.debug("Average: %s" % self.averageQueue[-1])
        # slope assume equal distance between points i.e divded by 1
        # Let 5 points accumulate, then take simple slope.
        if len(self.averageQueue) >= 3 and self.counter == 3:
            self.slopeQueue.append(self.averageQueue[-1] - self.averageQueue[-3])
            self.counter = 0
            logger.debug("Slope: %s" % self.slopeQueue[-1])

        # Slope change is confirmed if newest slope is not equal to last two slopes.
        # 3rd slope is popped
        if len(self.slopeQueue) >= 3:
            signChange = sign(self.slopeQueue[-1]) != sign(self.slopeQueue[-2]) and \
                         sign(self.slopeQueue[-1]) != sign(self.slopeQueue.pop(2))

            # Stop repeated signals
            return signChange
        else:
            return False


class Captain(Node):

    def __init__(self):
        super().__init__('captain')
        self.junctions = {}
        self.mapPublisher = self.create_publisher(String, 'navigationMap', 10)
        self.beaconSubscriber = self.create_subscription(String, 'beacons', self.readBeacon, 10)

    def passedBeacon(self, junction):
        # TODO: Determine how to turn at junction.
        mapUpdate = String()
        mapUpdate.data = "return"
        self.mapPublisher.publish(mapUpdate)

    def readBeacon(self, beacon):
        self.get_logger().debug('Received: "%s"' % beacon.data)

        if beacon.data.split(",")[0] in self.junctions:
            if self.junctions[beacon.data.split(",")[0]].addDataPoint(beacon.data.split(",")[1], self.get_logger()):
                self.get_logger().info('Passed beacon: "%s"' % beacon.data.split(",")[0])
                self.passedBeacon(beacon.data.split(",")[0])
        else:
            self.junctions[beacon.data.split(",")[0]] = JunctionSlopeTracker(10)
            self.junctions[beacon.data.split(",")[0]].addDataPoint(beacon.data.split(",")[1], self.get_logger())

        # TODO: Determine if this code can be removed or if it should be reachable
        if False:
            f = open('captainLog.csv', "a")
            f.write(beacon.data + "\n")
            f.close()


DEBUG = False


def main():
    rclpy.init()
    captain = Captain()
    # rclpy.spin(captain)

    while (DEBUG):
        mapUpdate = String()
        mapUpdate.data = input("Enter a navigational update: ")
        captain.mapPublisher.publish(mapUpdate)
        captain.get_logger().debug('Publishing: "%s"' % mapUpdate.data)

    rclpy.spin(captain)


if __name__ == '__main__':
    main()
