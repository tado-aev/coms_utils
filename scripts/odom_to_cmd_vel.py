#!/usr/bin/env python2

# Script to directly convert /odom topic to /cmd_vel

import rosbag

import sys

if len(sys.argv) < 2:
    sys.stderr.write('Not enough arguments\n')
    sys.exit(1)

out_fn = 'output.bag'
if len(sys.argv) > 2:
    out_fn = sys.argv[2]

bag = rosbag.Bag(sys.argv[1])
out_bag = rosbag.Bag(out_fn, 'w')

for topic, m, t in bag:
    if topic != '/odom':
        continue
    out_bag.write('/cmd_vel', m.twist)
