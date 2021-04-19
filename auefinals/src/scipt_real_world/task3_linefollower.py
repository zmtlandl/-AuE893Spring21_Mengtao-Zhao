#!/usr/bin/env python3

import roslib
import sys
import rospy
import cv2
import time
import numpy as np
from sensor_msgs.msg import LaserScan
from numpy import inf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from termcolor import colored
from darknet_ros_msgs.msg import BoundingBoxes
from move_robot import MoveTurtlebot3
# from task2_obstacleavoidance import *

kp = 0.08
kd = 0
ki = 0.0001
pre_error = 0
err_sum = 0
temp = 0
t1 = 0
first_line_confirmation = False
line_find_factor = 0
n = 0
stopped = False
already_stopped_once  = False
time_delay = 0
"""
camera_callback functions gets images from topic subscribed and converts opencv image format for   
next steps
1] crop image to form mask which will be used for lane centroid
2] convert from RGB image format to gray
3] if lane color present in mask, find x,y co-ordinates of color mask centroid.
4] if lane not found, invoke lane finding maneuver (twist_object.linear.x  = 0.1, twist_object.angular.z = 0.02)
5] if lane found, invoke lane following maneuver (proportional controller based on cx distance)
6] if was detected earlier and lane finding maneuver invokes, sprial motion of turtlebot will be  
implemented. 
"""

def traffic_sign_callback(data):
    global  stopped, already_stopped_once, time_delay  #,delta, linear_speed, person_in_camera_view
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    twist_object = Twist()

    for box in data.bounding_boxes:

        if box.Class == "stop sign" and not already_stopped_once:
            print(colored('STOP sign ahead',"red"))
            velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
            while time_delay < 2:
                # velocity_publisher.publish(Twist(Vector3(0.2,0,0), Vector3(0,0,0)))
                rospy.sleep(1)
                time_delay+=1

            for i in range(20,-5,-5):
                linear_speed = i * 0.01
                twist_object.linear.x = linear_speed
                twist_object.angular.z = 0
                cmd_vel_pub.publish(twist_object)
                time.sleep(0.5)
            stopped = True
            print(colored('Stopped for stop sign',"red"))
            
            time.sleep(3)
            print('Starting movement after stop sign')
            stopped = False
            already_stopped_once = True

            for i in range(0,25,5):
                linear_speed = i * 0.01
                twist_object.linear.x = linear_speed
                twist_object.angular.z = 0
                cmd_vel_pub.publish(twist_object)
                time.sleep(0.5)

        # elif box.Class == "person":
        #     print('Person in camera view.')
        #     person_in_camera_view = True
            
class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/raspicam_node/image_raw",Image,self.camera_callback)
        # self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        # self.moveTurtlebot3_object = MoveTurtlebot3()
        # rospy.on_shutdown(self.clean_shutdown)

    def camera_callback(self,data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        height, width, channels = cv_image.shape
        print("height:",height,"width:",width)
        crop_img = cv_image[int(height)-30:int(height)][1:int(width)]
        print("crop_img:",len(crop_img))

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Define the yellow section in HSV
        yellow_lower = np.array([20,100,100])
        yellow_higher = np.array([50,255,255])
        mask = cv2.inRange(hsv, yellow_lower, yellow_higher)
        M = cv2.moments(mask, False) 

        cv_image2 = cv_image

        global first_line_confirmation
        global t1,n
        global line_find_factor

        try:
            cx = int(M['m10']/M['m00'])
            cy = int(M['m01']/M['m00'])
            rospy.loginfo("Mode - Line following")
            # cv_image = cv2.putText(cv_image2,"Mode: Line following maneuver", (20,22),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
            invoke_line_finder = False
            first_line_confirmation = True
        except ZeroDivisionError:
            cx = height/2
            cy = width/2
            rospy.loginfo("no version")
            invoke_line_finder = True
            lane_detected      = False

        # Draw the centroid in the result image
        cv2.circle(mask, (int(cx),int(cy)), 10, (255,0,0),-1)

        global kp,ki,kd,pre_error,err_sum,temp,stopped
        err = cx - width/2 
        # err = cx - width/2 +300

        #define the nodes
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        twist_object = Twist()

        #avoid steady state oscillation
        if err > -5 and err < 5:
            err = 0
        
        if not stopped:
        # line following maneuver control/// why here use - err
            twist_object.angular.z = np.clip((-float(err)*kp/100 + kd*(-err+pre_error)/100),-0.2,0.2)
            a_temp                 = np.clip((-float(err)*kp/100 + kd*(-err+pre_error)/100),-0.2,0.2)

            twist_object.linear.x = 0.1 #np.clip(0.2*(1-abs(a_temp)/0.2),0,0.2)
            b_temp                = 0.1 #np.clip(0.2*(1-abs(a_temp)/0.2),0,0.2)


            # after each control
            pre_error = temp
            err_sum = err_sum + err
        else:
            twist_object.angular.z = 0
            twist_object.linear.x = 0

    
        # control to find lane
        t0 = float(rospy.Time.now().to_sec())
        timestep = (t0-t1)
        # print(colored("invoke_lane_finder:",'red'),invoke_line_finder)

        if invoke_line_finder:
            if first_line_confirmation == False:
                print("Wandering mode")
                cv_image2 = cv2.putText(cv_image2,"Wandering mode", (20,24),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
                twist_object.linear.x = 0.05
                twist_object.angular.z = 0.0

            if first_line_confirmation:
                twist_object.linear.x = 0.2
                twist_object.angular.z = np.clip((line_find_factor * 0.05),0,0.08)
                n = n + timestep*twist_object.angular.z

                if n > 3.14159*1.5 and invoke_line_finder:
                    n = 0
                    line_find_factor = line_find_factor + 1

                print("Mode - Line finding maneuver - 2")
                # cv_image2 = cv2.putText(cv_image2,"Mode: Line following maneuver-2", (20,24),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
                print("Angle turned by bot=>"+str(n))
        t1 = t0
        # if a_temp == twist_object.angular.z and b_temp == twist_object.linear.x:
        #     line_find_factor = 0
        #     n = 0
        cmd_vel_pub.publish(twist_object)

        
        cv2.namedWindow("Line Follower",cv2.WINDOW_NORMAL)
        cv2.imshow("Line Follower",cv_image2)
        cv2.resizeWindow('Line Follower', (700,700))
        cv2.moveWindow("Original", 30,30)
        cv2.waitKey(1)

        # rospy.loginfo("Angular Value Sent ===>" + str(twist_object.angular.z))
        # self.moveTurtlebot3_object.move_robot(twist_object)

    def clean_up(self):
        # self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

    def clean_shutdown(self):
        ''' Stop robot when shutting down '''

        rospy.loginfo("System is shutting down. Stopping robot...")
        velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
        velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
        


    
def main():
    
    
    rospy.init_node('line_following_node', anonymous=True)
    
    line_follower_object = LineFollower()
    rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes,traffic_sign_callback)
    rate = rospy.Rate(10)

    
    # while not rospy.is_shutdown():
    #     rate.sleep()

    ctrl_c = False
    def shutdownbot():
        
        line_follower_object.clean_up()
        rospy.loginfo("Shutdown the bot!")
        ctrl_c = True
        msg=Twist()
        msg.linear.x = 0
        msg.angular.z=0
        pub_= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pub_.publish(msg)  
    rospy.on_shutdown(shutdownbot)
    rospy.spin()

    

    

if __name__ == '__main__':
    try:
        main()
        
    except rospy.ROSInterruptException: pass