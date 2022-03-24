#!/usr/bin/env python
# @author: Emily Clarke

import csv, string

mapGraph = []

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
def expectedBeacon(mapgraph, sourceJunction, destJunction):
    for junction in mapGraph:
        if(junction[0] == destJunction):
            for beacon in junction[1]:
                if(beacon[1] == sourceJunction):
                    return beacon[0]
    return -1

# Returns the turn direction given a beaconID and the target junction
def turnDirection(mapGraph, beaconID, junctionID):
    turns = [
        ["u-turn", "left", "straight", "right"],
        ["right", "u-turn", "left", "straight"],
        ["straight", "right", "u-turn", "left"],
        ["left", "straight", "right", "u-turn"],
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


# Example
mapGraph = loadMap(mapGraph)
path = bfs(mapGraph,"1","13")
print("Path going from 1 to 13: " + str(path))
print("Starting going straight South from 1")

count = 0
for junction in path:
    print("At beacon " + str(expectedBeacon(mapGraph,junction,path[count+1])) 
        + ", junction " + str(beaconToJunction(mapGraph,expectedBeacon(mapGraph,junction,path[count+1]))))
    if (count+2 == len(path)):
        print("Arrived at destination!")
        break
    print("\t\t    ------> " + turnDirection(mapGraph,expectedBeacon(mapGraph,junction,path[count+1]),path[count+2])
        + " to junction " + path[count+2])
    count += 1
    


