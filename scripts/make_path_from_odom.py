#!/usr/bin/env python2

# Creats a path by recording the coordinates from Odometry.
# Use this with AMCL to get more accurate coordinates.

import rospy
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped

import sys
import threading

points = Path()
latest_odom = Odometry()

def odom_callback(msg):
    global latest_odom
    latest_odom = msg

rospy.init_node('make_path_from_odom')
rospy.Subscriber('odom', Odometry, odom_callback)
path_pub = rospy.Publisher('path', Path, queue_size=1)

def save_points():
    global points
    global latest_odom

    while not rospy.is_shutdown():
        line = sys.stdin.readline().strip()
        if line == 'exit' or line == 'quit' or line == 'q':
            path_pub.publish(points)
            print('Bye!')
            rospy.signal_shutdown('Terminated by user')
            sys.exit(0)
        p = PoseStamped()
        p.header = latest_odom.header
        p.pose = latest_odom.pose.pose
        points.poses.append(p)

if __name__ == '__main__':
    t = threading.Thread(name='save_points', target=save_points)
    t.daemon = True
    t.start()
    rospy.loginfo('Press enter to record point')
    rospy.loginfo('q[uit] or exit to publish and exit')

    rospy.spin()
