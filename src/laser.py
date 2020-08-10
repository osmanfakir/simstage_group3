#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

def callback(msg):
	print ("Value at 0 degree")
	print(msg.ranges[0])

	print ("Value at 90 degree")
	print(msg.ranges[100])

	print ("Value at 180 degree")
	print(msg.ranges[120])

rospy.init_node('scan_value')
sub = rospy.Subscriber('base_scan', LaserScan, callback)
rospy.spin()