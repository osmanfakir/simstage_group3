#!/usr/bin/env python2


import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

import turtlesim.srv
import rosservice
import numpy as np

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

        #print(len(msg.ranges))
        print ("Value at 0 degree")
        print(msg.ranges[0])
        print ("Value at 90 degree")
        print(msg.ranges[120])
        print ("Value at 180 degree")
        print(msg.ranges[239])

        self.ranges = np.array_split(msg.ranges, 6)
        print("right side", min(self.ranges[0]))
        print("right side one ", min(self.ranges[1]))
        print("mid point", min(self.ranges[2]))
        print("mid point one", min(self.ranges[3]))
        print("left side", min(self.ranges[4]))
        print("left side one", min(self.ranges[5]))

        self.right = min(self.ranges[0])
        self.right1 = min(self.ranges[1])
        self.mid = min(self.ranges[2])
        self.mid1 = min(self.ranges[0])
        self.left = min(self.ranges[1])
        self.left1 = min(self.ranges[2])

    def run_controller(self):
        rospy.loginfo("Controller is now active")

        rate = rospy.Rate(5)
        while not rospy.is_shutdown():
            #pretty straight forward move 
            if self.mid > 0.8:
                msg = Twist()
                msg.linear.x = 1
                msg.angular.z = 0
                self.controller_pub.publish(msg)
                rate.sleep()
            #pretty straight forward move 
            if self.mid > 0.8 and self.right < 1.5:
                msg = Twist()
                msg.linear.x = 1
                msg.angular.z = 0
                self.controller_pub.publish(msg)
                rate.sleep()
            #Turning right
            if self.mid < 0.8 and self.right > 1.6:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = -15
                self.controller_pub.publish(msg)
                rate.sleep()
            #Turning left
            if self.right1 < 1 and self.mid < .8 and self.left < .9:
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = 2
                self.controller_pub.publish(msg)
                rate.sleep()
            #Till here the code is okay.

            #Turning a bit right with right one.
            if self.right1 < 3.14 :
                msg = Twist()
                msg.linear.x = 0
                msg.angular.z = -.2
                self.controller_pub.publish(msg)
                rate.sleep()
            #Coming back from the blocked area
            if self.left < .7 and self.right < .99 and self.left < .7:
                msg = Twist()
                msg.linear.x = -2
                msg.angular.z = 0
                self.controller_pub.publish(msg)
                rate.sleep()
            

           
            

            

        rospy.loginfo("Controller is no longer active.")
    
        


if __name__ == '__main__':
    rospy.init_node('command_vel_pub')
    controller = Controller()
    controller.run_controller()

    rospy.spin()