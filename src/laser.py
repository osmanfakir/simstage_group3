#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def callback(msg):
	print ("value at 0 degree")
	print(msg.ranges[0])
	print ("value at 90 degree")
	print(msg.ranges[120])
	print ("value at 180 degree")
	print(msg.ranges[239])