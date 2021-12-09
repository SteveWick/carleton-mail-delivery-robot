import matplotlib.pyplot as plt
import numpy as np
import csv

def moving_average(x, w):
    return np.convolve(x, np.ones(w), 'valid') / w

states = []
distances = []
angles = []
action = []
beacons = {}
with open('/var/log/mailDeliveryRobot/driverLog.csv') as csvfile:
    reader = csv.reader(csvfile,delimiter=",")
    for row in reader:
        states.append(row[0])
        distances.append(int(float(row[1])))
        angles.append(int(float(row[2])))
        action.append(row[3])

with open('/var/log/mailDeliveryRobot/captainLog.csv') as csvfile:
    reader = csv.reader(csvfile,delimiter=",")
    for row in reader:
        if not row[0] in beacons.keys():
            beacons[row[0]] = []
        beacons[row[0]].append(row[1])


samples = list(range(1,len(states)+1))
fig,(distancePlot,anglePlot,statePlot,actionPlot) = plt.subplots(4,constrained_layout=True)
# fig.tight_layout()
fig.suptitle('Robot Driver')
distancePlot.plot(samples,distances)
distancePlot.set_title("Distance")
anglePlot.plot(samples,angles)
anglePlot.set_title("Angle")
statePlot.plot(samples,states)
statePlot.set_title("State")
actionPlot.plot(samples,action)
actionPlot.set_title("Action")
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