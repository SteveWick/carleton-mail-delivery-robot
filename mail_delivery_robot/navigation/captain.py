#!/usr/bin/env python
# @author: Stephen Wicklund and Emily Clarke

# SUBSCRIBER:   beacons
# PUBLISHER:    navigationMap
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import csv

# ~~~~ DEFAULTS ~~~~~
magicNumbers = {
    'BEACON_OUTLIER_THRESHOLD': 7,
}

# ~~~~ Load overrides ~~~~
def loadNumberOverrides():
    with open('/var/local/magicNumbers.csv') as csvfile:
        reader = csv.reader(csvfile,delimiter=",")
        for row in reader:
            magicNumbers[row[0]]= row[1]
    return magicNumbers

def sign(x):
    return bool(x > 0) - bool(x < 0)

class JunctionSlopeTracker():
    def __init__(self,N):
        self.dataQueue = []
        self.averageQueue = []
        self.slopeQueue = []
        self.N = N
        self.sum = 0

        self.counter = 0
        self.signalSent = False
    
    def addDataPoint(self,dataPoint,logger):
        if(len(self.averageQueue) != 0 and abs(int(dataPoint) - int(self.averageQueue[-1])) > int(magicNumbers['BEACON_OUTLIER_THRESHOLD'])):
            return False
        
        #Remove data point exceeding window size N
        if len(self.dataQueue) >= self.N:
            self.sum -= int(self.dataQueue.pop(0))

        #Add new data point and update sum and average queue
        self.dataQueue.append(dataPoint)
        self.sum += int(dataPoint)
        self.averageQueue.append(self.sum / len(self.dataQueue))
        self.counter += 1
        logger.debug("Average: %s" %  self.averageQueue[-1])
        #slope assume equal distance between points i.e divded by 1
        #Let 5 points accumulate, then take simple slope.
        if len(self.averageQueue) >= 3 and self.counter == 3:
            self.slopeQueue.append(self.averageQueue[-1] - self.averageQueue[-3])
            self.counter = 0
            logger.debug("Slope: %s" %  self.slopeQueue[-1])
        

        #Slope change is confirmed if newest slope is not equal to last two slopes.
        #3rd slope is popped
        if len(self.slopeQueue) >= 3:
            signChange = sign(self.slopeQueue[-1]) != sign(self.slopeQueue[-2]) and\
            sign(self.slopeQueue[-1]) != sign(self.slopeQueue.pop(2))
            
            #Stop repeated signals
            return signChange
        else:
            return False
  
# ~~~ Pathfinding ~~~
# Loads map from csv file
def loadMap(mapGraph):
    with open('/var/local/map.csv') as csvfile:
    # with open('map.csv') as csvfile:
        reader = csv.reader(csvfile,delimiter=",")
        for row in reader:
            mapGraph.append((row[0],((row[1],row[2]),(row[3],row[4]),(row[5],row[6]),(row[7],row[8]))))
    return mapGraph

# converts junction id to array id   i.e. junctions ["1","5","8"] maps to [0,1,2]
def idToVertexNum(mapGraph, id):
    count = 0
    for vertex in mapGraph:
        if vertex[0] == id:
            return count
        count += 1
    return -1

# breadth first search for shortest path in the graph. Returns array of junctionIDs of shortest path
def bfs(mapGraph, source, dest):
    root = idToVertexNum(mapGraph, source)

    visited = [False] * len(mapGraph)
    queue = []
    traceback = []
    traversal = []
    found = False

    queue.append(root)
    visited[root] = True

    while queue:
        root = queue.pop(0)
        traversal.append(root)

        for i in mapGraph[root][1]:
            if (i[1] == dest):
                traceback.append(i[1])
                found = True
                break
            num = idToVertexNum(mapGraph,i[1])
            if(num != -1 and visited[num] == False):
                queue.append(num)
                visited[num] = True
        
        if(found):
            traversal.reverse()
            for vertex in traversal:
                for i in mapGraph[vertex][1]:
                    if (i[1] == traceback[-1]):
                        traceback.append(mapGraph[vertex][0])
                        break
            traceback.reverse()
            # print("shortest path from " + source + " to " + dest + ": " + str(traceback))
            return traceback

# Returns the junctionID for a given beaconID
def beaconToJunction(mapGraph, beaconID):
    for junction in mapGraph:
        for beacon in junction[1]:
            if (beacon[0] == beaconID):
                return junction[0]
    return -1

# Returns the expected beacon if traveling from one junction to another
def expectedBeacon(mapGraph, sourceJunction, destJunction):
    for junction in mapGraph:
        if(junction[0] == destJunction):
            for beacon in junction[1]:
                if(beacon[1] == sourceJunction):
                    return beacon[0]
    return -1

# Returns the turn direction given a beaconID and the target junction
def turnDirection(mapGraph, beaconID, junctionID):
    turns = [
        ["u-turn", "Lturn", "PassThrough", "RTurn"],
        ["RTurn", "u-turn", "Lturn", "PassThrough"],
        ["PassThrough", "RTurn", "u-turn", "Lturn"],
        ["Lturn", "PassThrough", "RTurn", "u-turn"],
    ]
    for junction in mapGraph:
        count = 0
        sourceDirection = None
        destDirection = None
        for beacon in junction[1]:
            if (beacon[0] == beaconID):
                sourceDirection = count
            if (beacon[1] == junctionID):
                destDirection = count
            count += 1
            if (sourceDirection != None and destDirection != None):
                break
        if (sourceDirection != None and destDirection != None):
                break

    # print(turns[sourceDirection][destDirection])
    return turns[sourceDirection][destDirection]


class Captain(Node):

    def __init__(self):
        super().__init__('captain')
        self.beacons = {}
        self.mapPublisher = self.create_publisher(String,'navigationMap', 10)
        self.beaconSubscriber = self.create_subscription(String,'beacons', self.readBeacon,10)

    def passedBeacon(self,beacon):
        mapGraph = []
        mapGraph = loadMap(mapGraph)
        currJunc = beaconToJunction(mapGraph,beacon)
        path = bfs(mapGraph,currJunc, 2)
        self.get_logger().info('New path: "%s"' % path)
        turn = turnDirection(mapGraph,beacon,path[0])
        self.get_logger().info('Next turn: "%s"' % turn)
        mapUpdate = String()
        mapUpdate.data = turn
        self.mapPublisher.publish(mapUpdate)

    def readBeacon(self, beacon):
        self.get_logger().info('Received: "%s"' % beacon.data)
        
        # if beacon.data.split(",")[0] in self.beacons:
        #     if self.beacons[beacon.data.split(",")[0]].addDataPoint(beacon.data.split(",")[1], self.get_logger()):
        #         self.get_logger().info('Passed beacon: "%s"' % beacon.data.split(",")[0])
        #         self.passedBeacon(beacon.data.split(",")[0])
        # else:
        #     self.beacons[beacon.data.split(",")[0]] = JunctionSlopeTracker(10)
        #     self.beacons[beacon.data.split(",")[0]].addDataPoint(beacon.data.split(",")[1], self.get_logger())

        if (int(beacon.data.split(",")[1]) > -40):
            self.get_logger().info('Passed beacon: "%s"' % beacon.data.split(",")[0])
            self.passedBeacon(beacon.data.split(",")[0])

        if False:
            f = open('captainLog.csv', "a")
            f.write(beacon.data +"\n")
            f.close()

DEBUG = False
def main():
    try:
        loadNumberOverrides()
    except:
        print("No tuning file found!")
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