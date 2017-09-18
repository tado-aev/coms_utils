#!/usr/bin/env python2

import rospy
from nav_msgs.msg import Odometry

import math

def callback(data):
    lin = data.twist.twist.linear
    print(math.sqrt(lin.x**2 + lin.y**2))

rospy.init_node('show_vel')
sub = rospy.Subscriber('odom', Odometry, callback)

rospy.spin()
