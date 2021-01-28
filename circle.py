#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time

def poseCallback(pose_message):
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta


def move(velocity_publisher, linear_speed,angular_speed, is_forward):
        #declare a Twist message to send velocity commands
        velocity_message = Twist()
        #get current location 
 
        if (is_forward):
            velocity_message.linear.x =abs(linear_speed)
        else:
        	velocity_message.linear.x =-abs(linear_speed)


        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        
        while True :
                rospy.loginfo("Turtlesim moves forwards")
                velocity_message.angular.z =abs(angular_speed)
                velocity_publisher.publish(velocity_message)
                loop_rate.sleep()
                

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        move(velocity_publisher,1,1,True)
     
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
