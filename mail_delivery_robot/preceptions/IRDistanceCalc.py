#!/usr/bin/env python

# @author: Jozef Tierney and Devon Daley

# SUBSCRIBER:   none
# PUBLISHER:    String object to 'perceptions' node

import time
import math
from std_msgs.msg import String
import rclpy
from rclpy.node import Node
import sys
import Adafruit_ADS1x15


class IRSensor(Node):
    def __init__(self):
        super().__init__('ir_sensor')
        self.publisher_ = self.create_publisher(String, 'preceptions' , 10)
        timer_period = 0.2 #Seconds
        self.timer = self.create_timer(timer_period, self.sendReading)

    def sendReading(self):
        calc = String()
        # calc = calculate()
        calc.data = str(calculate())
        self.get_logger().debug('Publishing: "%s"' % calc)
        if calc.data == -1:
            pass
        else:
            self.publisher_.publish(calc)
            pass

# This script takes in inputs from an analog to digital converter connected to IR sensors and converts to cm before
#     using the two distance measurements to calculate the robot's distance from the wall. It sends wall distance to perceptions node.

# Import the ADS1x15 module.

# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()
stack1 = [0, 0, 0, 0, 0]
stack2 = [0, 0, 0, 0, 0]
avg1 = 0
avg2 = 0
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 2

# def test():
#     #this is just a test function to pass set values to check math
#     distance(5.5, 7.25)

def distance(dis1, dis2):
    #the two distance reading are passed to this from the sensors (in cm)
    #and this published the angle and distance values it calculates
    #set R as the angle between the two IR sensors
    R = 20
    #a = dis1, b = dis2
    r = math.sqrt(dis1 ** 2 + dis2 ** 2 - 2 * dis1 * dis2 * math.cos(R*math.pi/180))
    #B = math.asin((dis1*math.sin(R*math.pi/180))/r)*180/math.pi
    B = math.acos((r**2 + dis2**2 - dis1**2)/(2 * r * dis2)) * 180/math.pi
    A = 180 - B - R / 2
    #b = math.sin(B)*dis2/math.sin(A)
    b = math.sin(B*math.pi/180)*dis2/math.sin(A*math.pi/180)
    #print("Distance from wall is "+ str(b) +" centimeters")
    #print("Offset angle is "+ str(180 - A) +" degrees.")
    # When too close to wall, returns constant 10.2046768063
    if dis1 < 10.21 or dis2 < 10.21:
        b=1
        
    # out = "distance: " + str(abs(b)) + " angle: " + str(180 - A)
    out = str(abs(b)) +"," +str(180 - A)
    print(out)
    return out
    

#    when testing this printed nice headers but it's not needed anymore
#print('Reading ADS1x15 values, press Ctrl-C to quit...')
#print('| Sensor 1 | Sensor 2 |'.format(*range(2)))
#print('-' * 37)

def calculate():    
    values = [0]*2
    v = [0]*2
    for i in range(2):
        # Read the specified ADC channel using the gain value.
        #values[i] = 3.3*adc.read_adc(i, gain=GAIN)/33000
        v[i]= adc.read_adc(i, gain=GAIN)
        #this is the equation for the curve of inputs vs outputs to convert from input to cm
        values[i] = 5187878*v[i]**(-1.263763)
        #values[i] = adc.read_adc(i, gain=GAIN, data_rate=128)
        #distance(v[0], v[1])
    #test()
    #print('| {0:>6} | {1:>6} |'.format(*values))
        
    #this section deals with outliers, the program keeps the last 5 readings in a FIFO stack
    #when a new reading is collected it's compared to the average of the last 5, if is too much larger
    #or smaller than the average it ignores the value but still adds it to the stack
    #calculate averages
    avg1 = sum(stack1)/5
    avg2 = sum(stack2)/5
    
    #insert new values, pop oldest values.
    stack1.insert(0, values[0])
    stack1.pop(5)
    stack2.insert(0, values[1])
    stack2.pop(5)
    
    #prints values and averages for testing
    print("Sensor1: " + str(values[0]) + "    avg: " + str(avg1))
    print("Sensor2: " + str(values[1]) + "    avg: " + str(avg2))
    
    #check if the values are in the range and valid, this will likely need to be tuned in the future
    if values[0] < avg1*1.5 and values[0] > avg1*0.5:
        if values[1] < avg2*1.5 and values[1] > avg2*0.5:
            return distance(values[0], values[1])
    
    return -1
    
def main():
    rclpy.init()
    irsensor = IRSensor()

    # Run irsensor when not blocked by shutdown
    rclpy.spin(irsensor)
    


if __name__ == '__main__':
    main()

