#!/usr/bin/env python2

# Commands the accelerator percentage and measures the velocity change
# Usage:
#   accel_vel.py <output filename>
#
# Subscribes to the /encoder topic. If your encoder driver outputs pulse
# counts to a different topic, change it using your_topic_name:=encoder when
# running the node.
#
# Author: Naoki Mizuno (mizuno.naoki@rm.is.tohoku.ac.jp)
# License: MIT

import rospy
from coms_msgs.msg import ComsGAB, ComsEncoder

import signal
import sys
import math
import threading

class VelocityRecorder():
    WHEEL_DIAMETER = 0.5078
    COUNTS_PER_ROTATION = 2000

    def __init__(self, out_fn):
        self.fh = open(out_fn, 'w')
        self.is_writing_ = False
        self.initial_t = None
        self.prev_time = None
        self.prev_cnt = None
        self.enc_sub = rospy.Subscriber('encoder',
                                        ComsEncoder,
                                        self.encoder_callback)

    def encoder_callback(self, data):
        t = data.header.stamp
        if not self.initial_t and self.is_writing():
            self.initial_t = t
        cnt = data.count
        if not self.prev_cnt:
            self.prev_time = t
            self.prev_cnt = cnt
            return

        x = math.pi * VelocityRecorder.WHEEL_DIAMETER \
            * (cnt - self.prev_cnt) / VelocityRecorder.COUNTS_PER_ROTATION
        v = x / float((t - self.prev_time).to_sec())

        if self.is_writing():
            self.fh.write('{0} {1}\n'.format((t - self.initial_t).to_sec(), v))

        self.prev_time = t
        self.prev_cnt = cnt

    def is_writing(self):
        return self.is_writing_

    def start_writing(self):
        self.is_writing_ = True

    def stop_writing(self):
        self.is_writing_ = False

if len(sys.argv) < 2:
    sys.stderr.write('Not enough arguments\n')
    sys.exit(1)

rospy.init_node('accel_vel')
gab_pub = rospy.Publisher('cmd_gab', ComsGAB, queue_size=10)

recorder = VelocityRecorder(sys.argv[1])

def apply_brake():
    print('Stopping')
    recorder.stop_writing()
    msg = ComsGAB()
    msg.accel = 0
    msg.brake = 100
    msg.gear = 'd'
    gab_pub.publish(msg)

# Handle Ctrl-C
def signal_handler(signal, frame):
    apply_brake()
    sys.exit(0)
signal.signal(signal.SIGINT, signal_handler)

def get_input_and_publish():
    while not rospy.is_shutdown():
        line = sys.stdin.readline().strip()
        if line == '' or line == 'exit' or line == 'quit':
            apply_brake()
            print('Bye!')
            rospy.signal_shutdown('Terminated by user')
            sys.exit(0)

        try:
            val = float(line)
        except ValueError:
            sys.stderr.write('Not a number\n')
            continue
        if val < 0 or val > 100:
            sys.stderr.write('Value out of range [0, 100]\n')
            continue

        msg = ComsGAB()
        msg.accel = val
        msg.brake = 0
        msg.gear = 'd'
        gab_pub.publish(msg)
        recorder.start_writing()
        print('-> {0}%'.format(line))

t = threading.Thread(name='input_publish', target=get_input_and_publish)
t.daemon = True
t.start()

rospy.spin()
