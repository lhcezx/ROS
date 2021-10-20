#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author LUO haocheng

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
rospy.init_node('movement_corridor', anonymous=True) 
rate = rospy.Rate(10)

global stop_ # the variable decided if the robot need to stop
stop_ =  False 
case = '' # the variable to show which case we are
msg = Twist()

# The function to decide which situation we are in based on the laser value
def checkcase(range):
    linearx = 0 # linear speed
    angularz = 0 #angular speed
    global stop_
    
    #if there aren't any obstacles in front of the robot
    if ( range["center"] > 0.3 ):  
        case = 'NO OBSTACLE!'
        linearx = 0.2
        angularz = 0
    #if there is a obstacle in front of the robot    
    else:  
        if range["right"] > range["left"]: # if the right distance is bigger than the left, we turn to right
            linearx = 0
            angularz = -0.8
        elif range["right"] <= range["left"]:  # if the right distance is smaller than the left, we turn to left
            linearx = 0
            angularz = 0.8
            
        if (range["right"] > 0.4 and range["left"] > 0.4): # if the robot is in the exit, we need to stop it
            stop_ =  True
    
    if stop_:
        msg.linear.x = 0
        msg.angular.z = 0
        pub.publish(msg)
    else:    
        msg.linear.x = linearx
        msg.angular.z = angularz
        pub.publish(msg)
    
# The callback fonction for laser 
def callback(message):
    # range is a dictionary which stores the laser information in every direction
    range={
        "right" : np.mean(message.ranges[250:290]),
        "center" : min(min(message.ranges[0:25]) ,min(message.ranges[335:359]) , 1),
        "left" : np.mean(message.ranges[70:90]),
    }
    checkcase(range)
            
               
if __name__ == '__main__':
    
        try:
            pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1) 
            sub = rospy.Subscriber("/scan", LaserScan, callback) #creation d'un subscriber avec le type du data "Laserscan"
            rospy.spin()

        except rospy.ROSInterruptException:
            print("service failed")     

