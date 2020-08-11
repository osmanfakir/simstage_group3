#!/usr/bin/env python2


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import turtlesim.srv
import rosservice


class Controller:



    def __init__(self):

        # Pull arguments
        #self.robot_id = robot_id
        #print(self.robot_id)
        # Publishers definition
        self.controller_pub = rospy.Publisher(
            "cmd_vel",
            Twist, 
            queue_size = 1,)
        self.sub = rospy.Subscriber(
            "base_scan", 
            LaserScan, self.callback)
        

        rospy.sleep(0.25)
        

    def callback(self, msg):

        print(len(msg.ranges))
        print ("Value at 0 degree")
        print(msg.ranges[0])
        print ("Value at 90 degree")
        print(msg.ranges[120])
        print ("Value at 180 degree")
        print(msg.ranges[239])
        print ("Value at extr_right degree")
        print(msg.ranges[25])
        self.right = msg.ranges[10]
        self.extra_right = msg.ranges[25]
        self.middle = msg.ranges[120]
        self.left = msg.ranges[239]


    
    

    def run_controller(self):
        rospy.loginfo("Controller is now active")

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.middle > 2:
                msg = Twist()
                msg.linear.x = 5
                msg.angular.z = 0
                self.controller_pub.publish(msg)
                rate.sleep()

            if self.middle < 2:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 5
                self.controller_pub.publish(msg)
                rate.sleep()
            if self.middle < 2 and self.right > 2 and self.left< 2:
                msg = Twist()
                msg.linear.x = 1
                msg.angular.z = -1.7
                self.controller_pub.publish(msg)
                rate.sleep()
            if  self.extra_right > 3:
                msg = Twist()
                msg.linear.x = 2
                msg.angular.z = -3
                self.controller_pub.publish(msg)
                rate.sleep()
            
            

        rospy.loginfo("Controller is no longer active.")
    
        


if __name__ == '__main__':
    rospy.init_node('command_vel_pub')
    controller = Controller()
    controller.run_controller()

    rospy.spin()