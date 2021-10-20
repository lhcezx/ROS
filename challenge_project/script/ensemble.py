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
rospy.init_node('ensemble', anonymous=True) 
rate = rospy.Rate(100)

global obstacle # the variable to know if we are close to the obstacle
global color#the variable to know which track color we are, False: orange, True: cyan
global cylindre #the variable to avoid the cylindrical obstacle on the cyan trajectory
global tmp 
global full_mask #the variable which decide if we need full mask in image
global partie #the variable to decide which part we are
global nothing  # the variable to decide if we are out of the obstacle environment
global stop_ # the variable decided if the robot need to stop
global tmp_
tmp = 0
msg = Twist()
nothing = False 
stop_ = False
tmp_=0  
case = '' # a variable to show which case we are
full_mask = False
obstacle = False
color = False
cylindre = False
partie = 1

def tourne(a):
    msg.linear.x=0.
    msg.angular.z=a
    pub.publish(msg)

def avancer_corrige(a,err):
    msg.linear.x=a
    msg.angular.z=float(err) / 50
    pub.publish(msg)

def avancer(a):
    msg.linear.x=a
    msg.angular.z=0
    pub.publish(msg)

def stop():
    msg.linear.x=0
    msg.angular.z=0
    pub.publish(msg)   


def checkcase(range):
    global nothing
    global partie
    linearx = 0
    angularz = 0
    
    if partie == 2:
        #if we are finished the transition from part 1 to 2
        if tmp >= 12:
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
                    
                if (range["right"] > 0.4 and range["left"] > 0.4): # if the robot is in the exit, we are in part 3
                    partie = 3
              
            msg.linear.x = linearx
            msg.angular.z = angularz
            pub.publish(msg)
            
    elif partie ==3:
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
        if nothing == False:
            msg.linear.x = linearx
            msg.angular.z = angularz
            pub.publish(msg)


# callback function for laser
def callback2(message):
    # range is a dictionary which stores the laser information in every direction
    global obstacle
    global partie
    global tmp
    global cylindre
    
    if partie == 1:    
        if min(min(message.ranges[0:30]), min(message.ranges[345:359]))<0.4:
            obstacle=True
        else:
            obstacle=False
        
        if obstacle == False:  
            if min(message.ranges[295:359])>0.4:
                cylindre=False
            
            
    if partie == 2:
        #tmp is a global variable to transit the end of part 1 to part 2
        if tmp<6:
            tourne(-0.8)
            tmp+=1
        elif tmp<12:
            avancer(0.2)
            tmp+=1
        else:
            range={
                "left" : min(min(message.ranges[80:100]),1),
                "center" : min(min(message.ranges[0:25]), min(message.ranges[335:359]), 1),
                "right" : min(min(message.ranges[260:280]),1) 
            }
            checkcase(range)
    
    elif partie == 3:
        range={
            "right" : min(min(message.ranges[310:339]) , 1),
            "center" : min(min(message.ranges[0:20]) ,min(message.ranges[340:359]) , 1),
            "left" : min(min(message.ranges[21:50]) , 1),
            "nothing": min( min(message.ranges[70:110]), min(message.ranges[250:290]), min(message.ranges[160:200]))
        }       
        
        checkcase(range)


# Callback function for image processing
def callback1(message):
    global obstacle
    global color
    global cylindre 
    global full_mask
    global partie 
    global stop_
    global tmp_
    
    cvBridge = cv_bridge.CvBridge()
    # Transforme the image to openCV format, msg is the originial image form ros
    cvImage = cvBridge.imgmsg_to_cv2(message,desired_encoding='bgr8')
    
    # Change color represtation from BGR to HSV
    hsv = cv2.cvtColor(cvImage,cv2.COLOR_BGR2HSV)
    
    # Image binarisation 
    lower_cyan=np.array([78,43,46])
    upper_cyan=np.array([99,255,255])
    lower_orange=np.array([11,43,46])   
    upper_orange=np.array([25,255,255]) 
    lower_jaune=np.array([26,43,46])   
    upper_jaune=np.array([34,255,255]) 
    h, w, d = cvImage.shape  #height width and depth
    
    if partie == 1:
        
        if color:
            mask = cv2.inRange(hsv,lower_cyan,upper_cyan)
        else:        
            mask = cv2.inRange(hsv,lower_orange,upper_orange)
         
        
        search_top = 2*h/3
        search_left = w/4
        search_right = 3*w/4
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
            cv2.circle(mask,(cX,cY),8,(150),1) #8 pour le rayon du cercle et 150 pour la couleur
            cv2.imshow("image",mask)
            cv2.waitKey(3)
            
            err_y = (h-cY)
            err_x = (w/2)-cX
            if obstacle and color==False:  # there is an obstacle in front and we are on the orange trajectory
                stop()
            elif obstacle and color==True: # there is an obstacle in front and we are on the cyan trajectory
                tourne(0.8)
                cylindre = True
            elif obstacle==False and cylindre==True: # we are in cylinder case, we need to avoid it  
                avancer(0.2)  
            elif obstacle == False and cylindre == False :
                if (err_x)>20:  #centre cX is going to the left
                    tourne(0.6)    #turn to the left
                elif (err_x)<-20:  #centre cX is going to the right
                    tourne(-0.6)  #turn to the right
                else:
                    avancer_corrige(0.2,err_x)

            if(15<err_y<25):    # when the geometric center of the y is close to the border above, we change the trajectory color
                color=True
        
            if(abs(err_y<= 20) and full_mask == True):  # we are at the end of the cyan trajectory, go to part 2
                obstacle = False
                partie = 2
    
    elif partie == 3:
        mask = cv2.inRange(hsv,lower_jaune,upper_jaune)
        h,w = mask.shape  
        M = cv2.moments(mask)
    
        if M["m00"]>0:
        
            # Calculate x,y coordinate of center
            cX =int(M["m10"]/M["m00"])
            cY =int(M["m01"]/M["m00"])
            
            err_y = (h-cY)
            err_x = (w/2)-cX
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
                    avancer_corrige(0.2,err_x)
                    tmp_+=1   # we accumulate the time from zeros
                    if tmp_>30: # it time is over 30, we consider that the robot is more closer to the center now, so we stop it
                        stop_ = True
                    
            if stop_:
                stop()       
        
if __name__ == '__main__':
        try:
            pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10) 
            sub1 = rospy.Subscriber("/camera/image", Image, callback1)
            sub2 = rospy.Subscriber("/scan", LaserScan, callback2) #creation d'un subscriber avec le type du data "Laserscan"
            rospy.spin()
        except rospy.ROSInterruptException:
            pass
            
            
