#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#author LUO haocheng

import rospy
import numpy as np
import cv2
import cv_bridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
rospy.init_node('line_follow', anonymous=True) 
rate = rospy.Rate(100)

global obstacle # the variable to know if we are close to the obstacle
global color #the variable to know which track color we are, False: orange, True: cyan
global cylindre #the variable to avoid the cylindrical obstacle on the cyan trajectory
global full_mask #the variable which decide if we need full mask in image
global linear, angular # linear speed and angular speed
linear=0.22
angular=0.6
full_mask = False 
obstacle=False # False: No obstacle at the begin
color=False #False : we start with orange color
cylindre=False # False : There is not a cylindrical obstacle

# the function to stop the robot and turn with a angular speed
def tourne(a):
    vel_msg=Twist()
    vel_msg.linear.x=0.
    vel_msg.angular.z=a
    pub.publish(vel_msg)

#the function to make the robot move at a linear and a angular speed for the auto-correction
def avancer_corrige(a,err):
    vel_msg=Twist()
    vel_msg.linear.x=a
    vel_msg.angular.z=float(err) / 50
    pub.publish(vel_msg)

# the function to make the robot move at a constant linear speed a
def avancer(a):
    vel_msg=Twist()
    vel_msg.linear.x=a
    vel_msg.angular.z=0
    pub.publish(vel_msg)

# the function to stop the robot movement
def stop():
    vel_msg=Twist()
    vel_msg.linear.x=0
    vel_msg.angular.z=0
    pub.publish(vel_msg)   
   
# callback function for image processing
def callback1(msg):
    global obstacle
    global color
    global cylindre 
    global full_mask
    global linear, angular
    
    cvBridge = cv_bridge.CvBridge()
    # Transforme the image to openCV format, msg is the originial image form ros
    cvImage = cvBridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    # Change color represtation from BGR to HSV
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    
    # Image binarisation 
    lower_cyan=np.array([78,43,46])
    upper_cyan=np.array([99,255,255])
    lower_orange=np.array([11,43,46])   #orange color at min value
    upper_orange=np.array([25,255,255]) #orange color at max value
    
    if color:
        mask = cv2.inRange(hsv,lower_cyan,upper_cyan)
    else:        
        mask = cv2.inRange(hsv,lower_orange,upper_orange)
    h, w, d = cvImage.shape  #height width and depth
    
    # Masking the top of the image so that the robot will not deviate from the current trajectory
    search_top = 2*h/3
    #Covering the left and right sides of the image can ensure that the robot will not deviate from the current path due to seeing multiple paths
    search_left = w/4
    search_right = 3*w/4
    
    #when there is a cylinder in the path, we want to show all the mask, otherwise the variable full_mask = False
    if cylindre: full_mask = True
    if full_mask == False:
        mask[0:int(search_top), 0:int(w)] = 0
        mask[0:int(h), 0:int(search_left)] = 0
        mask[0:int(h), int(search_right):int(w)] = 0
        
 
    # Compute the mask moments
    M = cv2.moments(mask)
    if M["m00"]>0:
    
        # Calculate x,y coordinate of center
        cX =int(M["m10"]/M["m00"])
        cY =int(M["m01"]/M["m00"])
        # Display the image with cv.imshow, or draw a form with cv2.rectangle or cv2.circle
        cv2.circle(mask,(cX,cY),8,(150),1) #8 pour le rayon du cercle et 150 pour la couleur
        cv2.imshow("image",mask)
        cv2.waitKey(3)
        
        err_y = (h-cY)  # Calculate the difference between cY and the bottom of the image
        err_x = (w/2)-cX #Calculate the difference between cY and the median value of the x-axis of the picture
        if obstacle and color==False:  # there is an obstacle in front and we are on the orange trajectory
            stop()
        elif obstacle and color==True: # there is an obstacle in front and we are on the cyan trajectory
            tourne(angular)
            cylindre = True
        elif obstacle==False and cylindre==True: # we are in cylinder case, we need to avoid it  
            avancer(linear)  
        elif obstacle == False and cylindre == False : # there is no obstacle in front
            if (err_x)>20:  #centre cX is going to the left
                tourne(angular)    #turn to the left
            elif (err_x)<-20:  #centre cX is going to the right
                tourne(-angular)  #turn to the right
            else:
                avancer_corrige(linear,err_x)
        if(err_y<= 20 and full_mask == True):  # we are at the end of the cyan trajectory
            stop()
        elif 15<err_y<25 and color == False:    # when the geometric center of the y is close to the border above, we change the trajectory color
            color=True

# The callback fonction for laser
def callback2(msg):
    global obstacle
    global cylindre
    # We calculate the minimum radar value in a 60-degree sector in front of the robot
    if min(min(msg.ranges[0:30]), min(msg.ranges[345:359]))<0.4: 
        obstacle=True
    else:
        obstacle=False
    
    if obstacle == False:  
        # We calculate the minimum value in the sixty-degree sector on the right side of the robot, if it's bigger than 0.4, we will consider that we have already passsed the cylinder
        if min(msg.ranges[290:359])>0.4:
            cylindre=False
            

if __name__ == '__main__':
        try:
            pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
            sub1 = rospy.Subscriber("/camera/image", Image, callback1) #creation d'un subscriber avec le type du data "Image"
            sub2 = rospy.Subscriber("/scan", LaserScan, callback2) #creation d'un subscriber avec le type du data "Laserscan"
            rospy.spin()
            
        except rospy.ROSInterruptException:
            pass
            
            
