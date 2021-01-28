#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import time

def poseCallback(pose_message):
    global x
    global y, yaw
    x= pose_message.x
    y= pose_message.y
    yaw = pose_message.theta


def move(velocity_publisher, speed, distance, is_forward):
        #declare a Twist message to send velocity commands
        velocity_message = Twist()
        #get current location 
        global x, y
        x0=x
        y0=y

        if (is_forward):
            velocity_message.linear.x =abs(speed)
        else:
        	velocity_message.linear.x =-abs(speed)

        distance_moved = 0.0
        loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
        
        while True :
                rospy.loginfo("Turtlesim moves forwards")
                velocity_publisher.publish(velocity_message)

                loop_rate.sleep()
                
                distance_moved = abs(math.sqrt(((x-x0) ** 2) + ((y-y0) ** 2)))
                print  (distance_moved)
                print(x)
                if  not (distance_moved<distance):
                    rospy.loginfo("reached")
                    break
        
        #finally, stop the robot when the distance is moved
        velocity_message.linear.x =0
        velocity_publisher.publish(velocity_message)
    
def rotate (velocity_publisher, angular_speed, relative_angle, clockwise):
    
    velocity_message = Twist()

    if (clockwise):
        velocity_message.angular.z =-abs(angular_speed)
    else:
        velocity_message.angular.z =abs(angular_speed)

    angle_moved = 0.0
    loop_rate = rospy.Rate(10) # we publish the velocity at 10 Hz (10 times a second)    
    cmd_vel_topic='/turtle1/cmd_vel'
    velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)

    t0 = rospy.Time.now().to_sec()

    while True :
        rospy.loginfo("Turtlesim rotates")
        velocity_publisher.publish(velocity_message)

        t1 = rospy.Time.now().to_sec()
        current_angle = (t1-t0)*angular_speed
        loop_rate.sleep()
            
        if  (current_angle>relative_angle):
            rospy.loginfo("reached")
            break

    #finally, stop the robot when the distance is moved
    velocity_message.angular.z =0
    velocity_publisher.publish(velocity_message)

def setDesiredOrientation(publisher, angular_speed, desired_angle):
    relative_angle_radians = desired_angle - yaw
    clockwise=0
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print ("relative_angle_radians: ",relative_angle_radians)
    print ("desired_angle: ",desired_angle)
    rotate(publisher, angular_speed,abs(relative_angle_radians), clockwise)

def go_to_goal(velocity_publisher, x_goal, y_goal):
    global x
    global y, yaw

    velocity_message = Twist()

    while (True):
        K_linear = 0.5 
        distance = abs(math.sqrt(((x_goal-x) ** 2) + ((y_goal-y) ** 2)))

        linear_speed = distance * K_linear

        K_angular = 4.0
        desired_angle_goal = math.atan2(y_goal-y, x_goal-x)
        angular_speed = (desired_angle_goal-yaw)*K_angular

        velocity_message.linear.x = linear_speed
        velocity_message.angular.z = angular_speed

        velocity_publisher.publish(velocity_message)
        print ('x=', x, ', y=',y, ', distance to goal: ', distance)

        if (distance <0.01):
            break

def gridClean(publisher):
    
    go_to_goal(velocity_publisher,5,5)
    setDesiredOrientation(publisher, 0.2, 0)
    move(velocity_publisher, 0.5, 3.0, True)
    setDesiredOrientation(publisher, 0.2, math.pi/2)
    move(velocity_publisher, 0.5, 3.0, True)
    setDesiredOrientation(publisher, 0.2, math.pi)
    move(velocity_publisher, 0.5, 3.0, True)
    setDesiredOrientation(publisher, 0.2, -math.pi/2)
    move(velocity_publisher, 0.5, 3.0, True)
 

if __name__ == '__main__':
    try:
        
        rospy.init_node('turtlesim_motion_pose', anonymous=True)

        
        cmd_vel_topic='/turtle1/cmd_vel'
        velocity_publisher = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        
        position_topic = "/turtle1/pose"
        pose_subscriber = rospy.Subscriber(position_topic, Pose, poseCallback) 
        time.sleep(2)

        gridClean(velocity_publisher)

        
        
    except rospy.ROSInterruptException:
        rospy.loginfo("node terminated.")
