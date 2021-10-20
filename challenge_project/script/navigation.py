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
rospy.init_node('navigation', anonymous=True) 
rate = rospy.Rate(10)
global tmp_ # the variable to compte the time
global nothing # the variable to decide if we are out of the obstacle environment
global stop_  # the variable decided if the robot need to stop
msg = Twist()
nothing = False 
stop_ = False
tmp_=0  
case = '' # a variable to show which case we are

# the function to publish the speed message to topic /cmd_vel
def avancer_corrige(a,err): 
    msg.linear.x=a
    msg.angular.z=float(err) / 50 # we decide to let the robot move forward to the target with a corrected angular velocity
    pub.publish(msg)

def tourne(a):
    msg.linear.x=0.
    msg.angular.z=a
    pub.publish(msg)

def stop(): 
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg) 

# this function includes the different case depends on the laser information in three direction, it makes robot do the right decision in any time
def checkcase(range): 
    linearx = 0
    angularz = 0
    global nothing
    if( range["nothing"]>1.5):
        nothing = True
    elif ( range["right"] > 0.2  and range["center"] > 0.2 and range["left"] > 0.2):
        case = 'NO OBSTACLE!'
        linearx=0.2
        angularz=0
    elif ( range["right"] > 0.2  and range["center"] < 0.2 and range["left"] > 0.2 ):
        case = 'OBSTACLE CENTER!'
        linearx=0
        angularz=-0.5
    elif ( range["right"] < 0.2  and range["center"] > 0.2 and range["left"] > 0.2 ):
        case = 'OBSTACLE RIGHT!'
        linearx=0
        angularz=0.5
    elif ( range["right"] > 0.2  and range["center"] > 0.2 and range["left"] < 0.2 ):
        case = 'OBSTACLE LEFT!'
        linearx=0
        angularz=-0.5
    elif ( range["right"] < 0.2  and range["center"] > 0.2 and range["left"] < 0.2 ):
        case = 'OBSTACLE RIGHT AND LEFT!'
        linearx=0.15
        angularz=0
    elif ( range["right"] > 0.2  and range["center"] < 0.2 and range["left"] < 0.2 ):
        case = 'OBSTACLE CENTER AND LEFT!'
        linearx=0
        angularz=-0.5
    elif ( range["right"] < 0.2  and range["center"] < 0.2 and range["left"] > 0.2 ):
        case = 'OBSTACLE CENTER AND RIGHT!'
        linearx=0
        angularz=0.5
    elif ( range["right"] < 0.2  and range["center"] < 0.2 and range["left"] < 0.2 ):
        case = 'OBSTACLE AHEAD!'
        linearx=0
        angularz=0.8
    # if nothing == True that means we dont need laser anymore, but we need the camera to locate the center of the target
    if nothing == False:
        msg.linear.x = linearx
        msg.angular.z = angularz
        pub.publish(msg)
    
    
# the function concerning the image processing algorithm    
def callback1(msg): 
    global tmp_
    global stop_
    global nothing
    linear = 0.16
    
    cvBridge = cv_bridge.CvBridge()
    # Transforme the image to openCV format, msg is the originial image form ros
    cvImage = cvBridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    
    # Change color represtation from BGR to HSV
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    
    # Image binarisation 
    lower_jaune=np.array([26,43,46])   #yellow color at min value
    upper_jaune=np.array([34,255,255]) #yellow color at max value
         
    mask = cv2.inRange(hsv,lower_jaune,upper_jaune)
    h,w = mask.shape  #hauteur, largeur
 
    # Compute the mask moments
    M = cv2.moments(mask)
    if M["m00"]>0:
    
        # Calculate x,y coordinate of center
        cX =int(M["m10"]/M["m00"])
        cY =int(M["m01"]/M["m00"])
        
        err_x = (w/2)-cX
        cv2.circle(mask,(cX,cY),8,(150),1)
        cv2.imshow("image",mask)
        cv2.waitKey(3)
        
        # if there aren't any obstacle around of the robot, it moves to the target
        if nothing: 
            if (err_x)>10:  #centre cX is going to the left
                tourne(0.8)    #turn to the left
            elif (err_x)<-10:  #centre cX is going to the right
                tourne(-0.8)  #turn to the right
            else:
                avancer_corrige(0.2,err_x)
        # if the average of grayscale value of this area is over 200, it means we are very close to the target center
            if 200<np.mean(mask[int(h-80):int(h), int(w/2-40):int(w/2+40)]): 
                avancer_corrige(linear,err_x)
                tmp_+=1   # we accumulate the time from zeros
                if tmp_>30: # it time is over 30, we consider that the robot is more closer to the center now, so we stop it
                    stop_ = True
                
    if stop_:
        stop()

def callback2(message):
    # range is a dictionary which stores the laser information in every direction
    #min(message.ranges[0:20]), min(message.ranges[340:359]),
    range={
        "right" : min(min(message.ranges[310:339]) , 1), #right zone definition
        "center" : min(min(message.ranges[0:20]) ,min(message.ranges[340:359]) , 1), #center zone definition
        "left" : min(min(message.ranges[21:50]) , 1), #left zone definition
        "nothing": min( min(message.ranges[70:110]), min(message.ranges[250:290]), min(message.ranges[160:200])) # We look for the minimum value in the fan-shaped area on the left, right and back
    } 
    checkcase(range)

    

if __name__ == '__main__':
        try:
            pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
            sub1 = rospy.Subscriber("/camera/image", Image, callback1) #creation d'un subscriber avec le type du data "Image"
            sub2 = rospy.Subscriber("/scan", LaserScan, callback2) #creation d'un subscriber avec le type du data "Laserscan"
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
            
            
