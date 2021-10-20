#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import sys, termios, tty
import click
from geometry_msgs.msg import Twist

rospy.init_node('mybot_teleop', anonymous=True) #initialiser le node nomme 'mybot_teleop'
turtle_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10) #definir un publisher on the topic /turtle1/cmd_vel
rate = rospy.Rate(10) # 10hz #definir un nouveau rate pour publier le data


# Arrow keys codes
keys = {'\x1b[A':'up', '\x1b[B':'down', '\x1b[C':'right', '\x1b[D':'left', 's':'stop', 'q':'quit'}
if __name__ == '__main__':
    while not rospy.is_shutdown():

        try:    
            # Get character from console
            linear=rospy.get_param("/teleop/linear_scale") #get les parametres definit dans le fichier launch
            angular=rospy.get_param("/teleop/angular_scale")
            mykey = click.getchar()
            if mykey in keys.keys():
                char=keys[mykey]
                
            if char == 'up':    # UP key
                vel_msg=Twist()
                vel_msg.linear.x=linear
                turtle_vel_pub.publish(vel_msg)
                rospy.loginfo("publish turtle linear speed %f" %vel_msg.linear.x)
            if char == 'down':  # DOWN key
                vel_msg=Twist() 
                vel_msg.linear.x=-linear
                turtle_vel_pub.publish(vel_msg)
                rospy.loginfo("publish turtle linear speed %f" %vel_msg.linear.x)
            if char == 'left':  # RIGHT key
                vel_msg=Twist()
                vel_msg.angular.z=angular
                turtle_vel_pub.publish(vel_msg)
                rospy.loginfo("publish turtle angular speed %f" %vel_msg.angular.z)
            if char == 'right': # LEFT
                vel_msg=Twist()
                vel_msg.angular.z=-angular
                turtle_vel_pub.publish(vel_msg)
                rospy.loginfo("publish turtle angular speed %f" %vel_msg.angular.z)
            if char == "quit":  # QUIT
                break
            rospy.sleep(0.5)
            vel_msg.linear.x=0
            vel_msg.angular.z=0 
            turtle_vel_pub.publish(vel_msg) 
            rate.sleep()
                  

        except rospy.ROSInterruptException:
            pass
