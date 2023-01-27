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

TIMER_PERIOD = 0.2 #seconds


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
        self.pid_controller = PID() #init pid controller
        self.timer = self.create_timer(TIMER_PERIOD, self.sendReading)

    def sendReading(self):
        calc = String()
        
        #update pid controller and get output
        feedback = calculate()[0]
        self.pid_controller.update(feedback)
        output = self.pid_controller.output

        calc.data = str(output)

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

    # this is to find angle A, which tells us if the robot is moving towards or away from the wall
    # an obtuse angle (greater than 90 degrees) means it is moving away
    # an acute angle (less than 90 degrees) means it is moving towards
    angle_between_wall = math.acos(
        (sensor1_distance ** 2 + c ** 2 - sensor2_distance ** 2) / (2 * sensor1_distance * c)) * 180 / math.pi

    distance_from_wall = sensor1_distance * math.sin(angle_between_wall * math.pi / 180)

    distance_angle_calc = (abs(distance_from_wall),angle_between_wall)
    return distance_angle_calc
    
class PID:
    """PID Controller
    """

    def __init__(self, P=1.0, I=0.0, D=0.0, current_time=None):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.00
        self.current_time = current_time if current_time is not None else time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        """Clears PID computations and coefficients"""
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 20.0

        self.output = 0.0

    def update(self, feedback_value, current_time=None):
        """Calculates PID value for given reference feedback"""
        error = self.SetPoint - feedback_value

        self.current_time = current_time if current_time is not None else time.time()
        delta_time = self.current_time - self.last_time
        delta_error = error - self.last_error

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error
            self.ITerm += error * delta_time

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / delta_time

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error

            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm)

    def setKp(self, proportional_gain):
        """Determines how aggressively the PID reacts to the current error with setting Proportional Gain"""
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        """Determines how aggressively the PID reacts to the current error with setting Integral Gain"""
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        """Determines how aggressively the PID reacts to the current error with setting Derivative Gain"""
        self.Kd = derivative_gain

    def setWindup(self, windup):
        """Integral windup, also known as integrator windup or reset windup,
        refers to the situation in a PID feedback controller where
        a large change in setpoint occurs (say a positive change)
        and the integral terms accumulates a significant error
        during the rise (windup), thus overshooting and continuing
        to increase as this accumulated error is unwound
        (offset by errors in the other direction).
        The specific problem is the excess overshooting.
        """
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        """PID that should be updated at a regular interval.
        Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        """
        self.sample_time = sample_time


def main():
    rclpy.init()
    irsensor = IRSensor()

    # Run irsensor when not blocked by shutdown
    rclpy.spin(irsensor)
    
if __name__ == '__main__':
    main()


