#!/usr/bin/env python

# @author: Jozef Tierney and Devon Daley and Chase Scott

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
    """
    A class to represent the IR Sensors.

    ...

    Attributes
    ----------
    publisher : Publisher
        The classes publisher.
    timer : Timer
        The timer will be started and every timer_period_sec number of 
        seconds the provided callback function will be called.

    Methods
    -------
    sendReading():
        Publishes the reading from the IR sensors to the perceptions node.
    """
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


# Create an ADS1115 ADC (16-bit) instance.
adc = Adafruit_ADS1x15.ADS1115()
stack1 = [0, 0, 0, 0, 0]
stack2 = [0, 0, 0, 0, 0]
avg1 = 0
avg2 = 0
# See table 3 in the ADS1015/ADS1115 datasheet for more info on gain.
GAIN = 2

def calculate():
    '''
    Takes values recieved from the IR sensors and adds them to a stack. Then 
    when a new reading is collected it's compared to the average of the last 5,
    if it deviates too much from the mean it ignores the value but still adds it to the stack.

            Returns:
                    distance_angle_calc (str): A string tuple containing the distance_from_wall (cm)
                    and the angle_between_wall (deg) in the format: 'A,B'. Else returns -1
                    if it is an outlier measurment.
    '''        
    values = [0] * 2
    v = [0] * 2
    
    for i in range(2):
        # Read the specified ADC channel using the gain value.
        #values[i] = 3.3*adc.read_adc(i, gain=GAIN)/33000
        v[i] = adc.read_adc(i, gain=GAIN)
        #this is the equation for the curve of inputs vs outputs to convert from input to cm
        values[i] = 5187878*v[i]**(-1.263763)

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
    
    #check if the values are in the range and valid
    #TODO previous groups have mentioned that this could use tuning
    if values[0] < avg1*1.5 and values[0] > avg1*0.5:
        if values[1] < avg2*1.5 and values[1] > avg2*0.5:
            return distance(values[0], values[1])
    
    return -1

def distance(sensor1_distance, sensor2_distance):
    '''
    Returns the distance the robot is from the wall and its current angle.
    The diagram explaining the relation between variable names and the IR sensors
    can be found in the documentation.
    
            Parameters:
                    sensor1_distance (float): Distance reading (cm) from IR sensor 1
                    sensor2_distance (float): Distance reading (cm) from IR sensor 2

            Returns:
                    distance_angle_calc (str): A string tuple containing the distance_from_wall (cm)
                    and the angle_between_wall (deg) in the format: 'A,B'
    '''

    # set angle_between_sensors as the angle between the two IR sensors
    angle_between_sensors = 20

    # a is the distance between where the two censors make contact with the wall
    c = math.sqrt((sensor1_distance ** 2 + sensor2_distance ** 2) - (2 * sensor1_distance * sensor2_distance *
                                                                     math.cos(angle_between_sensors * math.pi / 180)))
    # h is the height of the triangle
    h = sensor1_distance * math.sin(angle_between_sensors * math.pi / 180)

    # B is an angle we need to find distance from wall
    B = math.asin(h / c)

    # this is to get the robots actual distance from the wall it is following
    distance_from_wall = sensor2_distance * math.sin(B)

    # this is to find angle A, which tells us if the robot is moving towards or away from the wall
    # an obtuse angle (greater than 90 degrees) means it is moving away
    # an acute angle (less than 90 degrees) means it is moving towards
    angle_between_wall = math.acos(
        (sensor1_distance ** 2 + c ** 2 - sensor2_distance ** 2) / (2 * sensor1_distance * c)) * 180 / math.pi

    distance_angle_calc = str(abs(distance_from_wall)) + "," + str(angle_between_wall)
    print(distance_angle_calc)
    return distance_angle_calc
    
def main():
    rclpy.init()
    irsensor = IRSensor()

    # Run irsensor when not blocked by shutdown
    rclpy.spin(irsensor)
    
if __name__ == '__main__':
    main()


