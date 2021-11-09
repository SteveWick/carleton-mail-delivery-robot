#!/usr/bin/env python

# @author: Gabriel Ciolac

# SUBSCRIBER:   none
# PUBLISHER:    String object to 'beacons' node

import rospy
from bluepy.btle import Scanner
from std_msgs.msg import String
from reader import BeaconReader

# This script is intended to transmit raw beacon distances to the perception translator

averages = dict()

'''
    Read RSSIs using bluepy library,
    returns RSSIs in a dictionary with MAC addresses as the key
'''
def __read_RSSI():
    reader = BeaconReader()
    ble_list = Scanner().scan(0.2)
    found_beacons = dict()
    for dev in ble_list:
        if dev.addr in reader.read_beacons().keys():
            found_beacons[dev.addr] = dev.rssi
    return found_beacons



'''
    Calculates distances from given RSSIs
    returns dictionary of distances with MAC addresses as the keys
'''
def __get_distance():
    
    reader = BeaconReader()
    read_rssi = __read_RSSI()
    MAC_ADDRs = read_rssi.keys()
    if len(MAC_ADDRs) == 0:
        return None

    distances = dict()
    for MAC in MAC_ADDRs:
        distances[MAC] = pow(10,(reader.read_beacons()[MAC][1] - read_rssi[MAC])/(10*reader.read_beacons()[MAC][0]))
    
    return distances

def __calc_average(mac, distance):
    global averages
    
    averages[mac].insert(0, distance)
    averages[mac].pop(5)
    avg = sum(averages[mac])/5
    if distance < avg * 1.25 and distance > avg * 0.75:
        return True
    return False

def rosMain():
    global averages
    
    pub = rospy.Publisher('beacons', String, queue_size=5)
    rospy.init_node('bluetoothBeacons', anonymous=True)
    rate = rospy.Rate(10)
    
    reader = BeaconReader()
    beacons = reader.read_beacons()
    
    while not rospy.is_shutdown():
        
        # Initialize values
        samples = dict()
        sumDist = dict()
        prevdist = dict()
        
        # Set default values for dicts
        for mac in beacons.keys():
            sumDist[mac] = 0
            samples[mac] = 10
        
        # Take 10 samples, get sum
        count = 0
        valid = True
        while (count < 10):
            distances = __get_distance()
            
            if not distances == None:
                outlier = False
                # Check all distances of macs, see if an obvious outlier
                for mac in distances.keys():
                    try:
                        if abs(distances[mac] - prevdist[mac]) > 1:
                            samples[mac] -= 1
                            outlier = True
                    except KeyError:
                        pass
                    except TypeError:
                        pass
                
                if not outlier:
                    # Get sum for every different beacon
                    for mac in distances.keys():
                        prevdist[mac] = distances[mac]
                        sumDist[mac] = sumDist[mac] + distances[mac]
                rate.sleep()
                count += 1
            else:
                valid = False
        
        if valid:
            # Find average of 10 samples (1 per second)
            avg = dict()
            for mac in sumDist:
                if not samples[mac] == 0: 
                    avg[mac] = sumDist[mac]/samples[mac]
                else:
                    avg[mac] = 100
                if avg[mac] == 0:
                    avg[mac] = 100
            
            # Construct message to publish
            MAC_ADDRs = avg.keys()
            to_send = ''
            for MAC in MAC_ADDRs:
                rospy.loginfo('Beacon: '+str(MAC)+','+str(avg[MAC])+'m')
                to_send = to_send +str(MAC)+','+str(avg[MAC]) + '\n'
            
            # If at least 1 beacon wasnt an outlier, then publish
            if not to_send == '':
                pub.publish(to_send)

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass



