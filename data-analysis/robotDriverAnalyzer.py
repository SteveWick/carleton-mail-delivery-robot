import matplotlib.pyplot as plt
import csv

states = []
distances = []
angles = []
action = []
with open('driverLog.csv') as csvfile:
    reader = csv.reader(csvfile,delimiter=",")
    for row in reader:
        states.append(row[0])
        distances.append(int(float(row[1])))
        angles.append(int(float(row[2])))
        action.append(row[3])

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