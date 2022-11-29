#!/usr/bin/env python
# @author: Stephen Wicklund

# SUBSCRIBER:   preceptions
# PUBLISHER:    command to actions
import rospy
from std_msgs.msg import String

def rosMain():
    rospy.init_node('handleCollision', anonymous=True)
    
    # creates publisher to actions 
    publisher = rospy.Publisher('actions', String, queue_size=10)

    # no actual collision behaviour in this file 
    # TODO: add collision handling behaviour to this file  

if __name__ == '__main__':
    try:
        rosMain()
    except rospy.ROSInterruptException:
        pass
