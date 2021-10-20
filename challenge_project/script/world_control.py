#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from gazebo_msgs.msg import ModelState, ModelStates

if __name__ == '__main__':
    try:
        # Node name
        rospy.init_node('gazebo_models_animation', anonymous=False)

        # Lets define a publisher on the topic_name topic (Twist message)
        # When a connection is latched, a reference to the last message published is saved and sent to any future subscribers that connect. This is useful for slow-changing or static data like a map. 
        pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=1, latch=True) 
        msg = ModelState()
        msg.model_name = "portail"
        # pub.publish(msg)
        rospy.sleep(1)

        current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)

        # Find the index of the model in the list of models published on the topic model_states
        ind = current_state.name.index(msg.model_name)

        
        while not rospy.is_shutdown():

            rospy.loginfo("Stay")
            current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)   # store the current position to set it in the next published message afin d'eviter les sauts de position
            msg.pose = current_state.pose[ind]
            msg.twist.linear.x = 0.0
            pub.publish(msg)
            rospy.sleep(10)


            vitesse=0.5
            rospy.loginfo("MOVING Away")
            current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)   # store the current position to set it in the next published message afin d'eviter les sauts de position
            msg.pose = current_state.pose[ind]
            msg.twist.linear.x = vitesse
            pub.publish(msg)
            rospy.sleep(1)

            rospy.loginfo("Stay")
            current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)   # store the current position to set it in the next published message afin d'eviter les sauts de position
            msg.pose = current_state.pose[ind]
            msg.twist.linear.x = 0.0
            pub.publish(msg)
            rospy.sleep(10)

            rospy.loginfo("MOVING Closer")  
            current_state = rospy.wait_for_message('/gazebo/model_states', ModelStates)
            # store the current position to set it in the next published message afin d'eviter les sauts de position
            msg.pose = current_state.pose[ind] 
            msg.twist.linear.x = -vitesse
            pub.publish(msg)                       
            rospy.sleep(1)




    except rospy.ROSInterruptException:
        pass
