#!/usr/bin/env python


## @package exproblab_ass1
#   \file turtle_controller.py
#   \brief This node control the investigator (turtle)
#   \author lorenzo Causa
#   \version 1.0
#   \date 28/10/2021
#
#   \details
#
#   Clients : <BR>
#
#   Subscriber: <BR>
#
#   /turtle1/pose
#
#   Publisher:<BR>
#
#   /turtle1/cmd_vel
#
#   Services: <BR>
#
#   /move_service
#
#   /search_hint_service
#          
# Description:    
# 
# This node provide the control of the turtlebot which simulate the behavior in the
# enviroment of the investigator
# 
#  

import roslib
import rospy
import smach
import smach_ros
import time
import random
from armor_msgs.srv import * 
from armor_msgs.msg import * 
from exproblab_ass1.srv import *
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

def target_coord(room):
    """ Returns the position of a given room, the positions are more or less the same as the ones in the real Cluedo game."""	
    x_target=0
    y_target=0
    
    if(room=='Lounge'):
        x_target=1
        y_target=1
    if(room=='Dining Room'):
        x_target=1
        y_target=5.5
    if(room=='Kitchen'):
        x_target=1
        y_target=10
    if(room=='Ballroom'):
        x_target=5.5
        y_target=10
    if(room=='Conservatory'):
        x_target=10
        y_target=10
    if(room=='Billiard Room'):
        x_target=10
        y_target=7
    if(room=='Library'):
        x_target=10
        y_target=4
    if(room=='Study'):
        x_target=10
        y_target=1
    if(room=='Hall'):
        x_target=5.5
        y_target=1    
    if(room=='Cluedo Room'):
        x_target=5.5
        y_target=5.5                              
            

    return [x_target,y_target]

    
def search_hints(req):
    """Simulate the searching of hints in a room by rotating our investigator (the turtle)"""	
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
    vel_msg = Twist()
    vel_msg.linear.x = 0 
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    T=0
    T_tot =1+random.random()
    delta_t=0.1
    
    while T<T_tot:
        vel_msg.angular.z =5	
        velocity_publisher.publish(vel_msg)
        time.sleep(delta_t)	
        T=T+delta_t
        
    vel_msg.angular.z =0  
    velocity_publisher.publish(vel_msg)
    time.sleep(0.2)  
    return True
    
def move(req):
    """ Function to simulate the movements of the investigator. """
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel',Twist, queue_size=10)
    [x_target,y_target] = target_coord(req.room)
    vel_msg = Twist()  
    distance_tolerance = 0.05
    
    while euclidean_distance(x_target,y_target) >= distance_tolerance:
        # Porportional controller.
        # Linear velocity in the x-axis.
        vel_msg.linear.x = linear_vel(x_target,y_target)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        # Angular velocity in the z-axis.
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = angular_vel(x_target,y_target)
        
        # Publishing our vel_msg
        velocity_publisher.publish(vel_msg)
        
        #print for debug
        #print('room: ',room,'ang vel: ',vel_msg.angular.z,'lin vel: ',vel_msg.linear.x )
        # Publish at the desired rate.
        time.sleep(0.1)

    # Stopping our robot after the movement is over.
    vel_msg.linear.x = 0
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)    
    
    return True

def update_pose(data):
    """Callback function which is called when a new message of type Pose is received by the subscriber."""
    global current_pose
    current_pose = data
    return    

def euclidean_distance(x_target,y_target ):
    """Euclidean distance between current pose and the goal."""
    return math.sqrt(pow((x_target - current_pose.x), 2) + pow((y_target - current_pose.y), 2)) 
    
def linear_vel(x_target,y_target, constant=1.5):
    """compute the correct linear vel"""
    return constant *euclidean_distance(x_target,y_target)   
    
def steering_angle (x_target,y_target):
    """compute the correct steering angle"""
    return math.atan2(y_target - current_pose.y, x_target- current_pose.x)      
    
def angular_vel(x_target,y_target, constant=10):
    """compute the correct angular vel"""
    return constant * normalize_angle(steering_angle(x_target,y_target) - current_pose.theta)      
    
def normalize_angle(angle):
    """"It normalize the target angle"""
    if(abs(angle) > math.pi):
        angle = angle - (2 * math.pi * angle) / (abs(angle))
    return angle     
    
def main():
    """ main of the turtle_controller node."""
    rospy.init_node('turtle_controller')
    #Create subscriber
    pose_subscriber = rospy.Subscriber('/turtle1/pose',Pose, update_pose)
    #create service
    turtle_move_service = rospy.Service('move_service', turtle_controller_srv, move)
    turtle_search_hints_service = rospy.Service('search_hint_service', turtle_controller_srv, search_hints)
    
    rospy.spin()
    sis.stop()


if __name__ == '__main__':
    main()
    
