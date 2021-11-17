#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   preceptions
# PUBLISHER:    command to actions
import rospy
from std_msgs.msg import String

def rosMain():
    rospy.init_node('handleCollision', anonymous=True)
    publisher = rospy.Publisher('actions', String, queue_size=10)
    # subscriber to preceptions
    rate = rospy.Rate(5)
    
    while not rospy.is_shutdown():
        command = -1
        if command == -1:
            pass
        else:
            publisher.publish(command)
        rate.sleep()

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass