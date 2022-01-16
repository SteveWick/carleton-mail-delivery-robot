import matplotlib.pyplot as plt
import numpy as np
import csv
import sys

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w

start = 0
end = sys.maxsize
if len(sys.argv) == 3:
    start = int(sys.argv[1])
    end = int(sys.argv[2])

print(len(sys.argv))
print("Start: " + str(start) + " End: " + str(end))

states = []
distances = []
angles = []
action = []
time = []
beacons = {}
counter = 0
with open('/var/log/mailDeliveryRobot/driverLog.csv') as csvfile:
    reader = csv.reader(csvfile,delimiter=",")
    for row in reader:
        if counter == 0:
            startTime = float(row[4])
        if counter < end and counter > start:
            states.append(row[0])
            distances.append(int(float(row[1])))
            angles.append(int(float(row[2])))
            action.append(row[3])
            time.append(float(row[4]) - startTime)
        counter += 1


with open('/var/log/mailDeliveryRobot/captainLog.csv') as csvfile:
    reader = csv.reader(csvfile,delimiter=",")
    for row in reader:

        if not row[0] in beacons.keys():
            beacons[row[0]] = []
        beacons[row[0]].append(row[1])


samples = list(range(1,len(states)+1))
fig,(distancePlot,anglePlot,statePlot,actionPlot) = plt.subplots(4,constrained_layout=True,figsize=(10,10))
# fig.tight_layout()
fig.suptitle('Robot Driver')
distancePlot.plot(time,distances)
distancePlot.set_title("Distance")
distancePlot.set_xlabel("Time (seconds)")
distancePlot.set_ylabel("Distance from wall (cm)")
distancePlot.grid()
anglePlot.plot(time,angles)
anglePlot.set_title("Angle")
anglePlot.grid()
anglePlot.set_xlabel("Time (seconds)")
anglePlot.set_ylabel("Angle to wall (degrees)")
statePlot.step(time,states)
statePlot.set_title("State")
statePlot.grid()
statePlot.set_xlabel("Time (seconds)")
statePlot.set_ylabel("Wall-following state")
actionPlot.step(time,action)
actionPlot.set_title("Action")
actionPlot.grid()
actionPlot.set_xlabel("Time (seconds)")
actionPlot.set_ylabel("Action sent")
fig.savefig('driverPlot.png')
plt.close()
beaconNum = 0

for beacon in beacons:
    rawBeaconData = beacons[beacon]
    print(rawBeaconData)
    rawBeaconData = [float(beacon) for beacon in rawBeaconData]
    rawBeaconData = moving_average(rawBeaconData,6)
    samples = list(range(1,len(rawBeaconData)+1))
    fig2 = plt.plot(samples,rawBeaconData)
    figName = str("BeaconPlot" + str(beaconNum) + ".png")
    plt.savefig(figName)