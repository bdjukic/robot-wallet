#!/usr/bin/env python

import rospy
import random

from std_msgs.msg import String
from nav_msgs.msg import Odometry

def fakeGpsLocator():
    gpsLocationPublisher = rospy.Publisher('gpsLocation', Odometry, queue_size=10)
    
    rospy.init_node('gpsTracker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():

        odom = Odometry()
        
        odom.pose.pose.position.x = random.uniform(10, 50)
        odom.pose.pose.position.y = random.uniform(10, 50)
        
        gpsLocationPublisher.publish(odom)
        
        rate.sleep()

if __name__ == '__main__':
    try:
        fakeGpsLocator()
    except rospy.ROSInterruptException:
        pass