#!/usr/bin/env python3

import cv2
import math
import time
import rospy
import numpy as np
import random
from numpy import inf
from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg  import Twist, Vector3
from darknet_ros_msgs.msg import BoundingBoxes
from apriltag_ros.msg import AprilTagDetection,AprilTagDetectionArray
# from pupil_apriltags import Detector
# from move_robot import MoveTurtlebot3

rospy.loginfo('tb3_gazebo_aue20_ms node started')
rospy.loginfo('wait for other processes to finish - start')
time.sleep(15)
rospy.loginfo('wait for other processes to finish - end')

delta        = 0
linear_speed = 0.1
cvbridge     = CvBridge()
camera_control  = 0
kp = 0.08
kd = 0
ki = 0.0001
pre_error = 0
stopped = False
already_stopped_once  = False
tag_detected = False
pass_stop_sign = False
# moveTurtlebot3_object = MoveTurtlebot3()


def obj_detector_callback(data):
    global delta, linear_speed, stopped, already_stopped_once,pass_stop_sign
    
    for box in data.bounding_boxes:

        if box.Class == "stop sign" and not already_stopped_once:
            print('STOP sign ahead')
            time.sleep(10)
            for i in range(10,-5,-5):
                linear_speed = i * 0.01
                time.sleep(1)
            stopped = True
            print('Stopped for stop sign')
            
            time.sleep(3)
            print('Starting movement after stop sign')
            stopped = False
            already_stopped_once = True
           

            for i in range(0,10,5):
                linear_speed = i * 0.01
                time.sleep(1)
            time.sleep(5)
            pass_stop_sign = True

def camera_callback(data):
    global cvbridge,delta,camera_control,stopped,pre_error,tag_detected
    cv_image = cvbridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    
    height, width, channels = cv_image.shape
    crop_img = cv_image[int(height)-30:int(height)][1:int(width)]
    hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

    yellow_lower = np.array([20,100,100])
    yellow_higher = np.array([50,255,255])
    mask = cv2.inRange(hsv, yellow_lower, yellow_higher)
    M = cv2.moments(mask, False) 


    try:
        cx = int(M['m10']/M['m00'])
        cy = int(M['m01']/M['m00'])
        # print("cx:",cx)
        
    except ZeroDivisionError:
        camera_control = 0
        return

    
    camera_control += 1

    # cv2.namedWindow("Original",cv2.WINDOW_NORMAL)
    # cv2.imshow("Original", cv_image)
    # cv2.resizeWindow("Original", (700,700))
    # cv2.waitKey(1)

    
    global kp,kd,pre_error,stopped
    err = cx - width/2 +300

    #avoid steady state oscillation
    if not stopped:
        delta = np.clip((-float(err)*kp/100 + kd*(-err+pre_error)/100),-0.2,0.2) if err < -2 or err > 2 else 0
        print("line_follow")
    else:
        delta = 0

def laser_callback(data):
    if camera_control == 0 and tag_detected == False:
        
        global delta

        Head_on_threshold        = 0.4
        side_obstacle_confidence = 0.7
        x  = list(data.ranges)

        f_l      = np.array(x[0:30])
        f_l[f_l  == inf] = 3.5
        f_l_dist = sum(f_l)/len(f_l)

        f_r      = np.array(x[330:360])
        f_r[f_r  == inf] = 3.5
        f_r_dist = sum(f_r)/len(f_r)

        l      = np.array(x[30:90])
        l[l    == inf] = 3.5
        l_dist = sum(l)/len(l)

        r = np.array(x[270:330])
        r[r == inf] = 3.5
        r_dist = sum(r)/len(r)	

        delta = l_dist - r_dist

        if min(f_l_dist, f_r_dist) < 3.5: # obstacle ahread
            print("wandering")
            # print ("f_l_dist: %.2f f_r_dist: %.2f delta: %.2f" %(f_l_dist,f_r_dist,delta))
            approach  = f_l_dist - f_r_dist
            
            if approach > 1: # Obstacle on the right
                delta = min(3.5,delta + 1/f_r_dist)
            elif approach < -1: #Obstacle on the left
                delta = max(-3.5, delta - 1/f_l_dist)
            else: 

                if min(f_l_dist,f_r_dist)<Head_on_threshold: 
                    print("ALert")
                    if -1*side_obstacle_confidence<delta<side_obstacle_confidence: 
                        if f_l_dist < f_r_dist:
                            delta = -3.5
                        elif f_l_dist > f_r_dist:
                            delta = +3.5
                        else:
                            delta = random.choice([-3.5,3.5]) 
                    else:
                        delta = np.sign(delta) * 3.5
        delta = 0.8*delta

def April_callback(data):
    global delta,linear_speed,tag_detected,camera_control
    if camera_control == 0:
        try:
            rotate = data.detections[0].pose.pose.pose.position.x - 0.089
            forward = data.detections[0].pose.pose.pose.position.z
            print("forward:",forward)
            # twist_object = Twist()
            
            Kp_rotate = 10
            count = 0
            if forward < 0.3:
                linear_speed = 0
                
                tag_detected = True
                return
                
            elif forward == 10000:
                linear_speed = 0
                tag_detected = True
            else:
                linear_speed = 0.1
                if rotate < 0:
                    count = 1
                if rotate > 0:
                    count = -1
                delta = count * Kp_rotate * abs(rotate) / 1.9
                tag_detected = True
                
            
        except IndexError:
            return

def default_callback(data):
        
    cv_image = CvBridge().imgmsg_to_cv2(data, desired_encoding="bgr8")
    
    # cv2.namedWindow("Scan",cv2.WINDOW_NORMAL)
    # cv2.imshow("Scan", cv_image)
    # cv2.resizeWindow("Scan", (700,700))
    # cv2.waitKey(1)


                


    
def pd_controller():

    global delta
    global linear_speed
    global kp,kd

    #Setup
    rospy.init_node('pd_control', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # image = rospy.Subscriber('/camera/rgb/image_raw', Image, default_callback)
    scan_subscriber = rospy.Subscriber('/scan', LaserScan, laser_callback)
    image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image, camera_callback)
    obj_detector_sub = rospy.Subscriber("/darknet_ros/bounding_boxes",BoundingBoxes, obj_detector_callback)
    April_tag = rospy.Subscriber("/tag_detections",AprilTagDetectionArray,April_callback)
    
    rate = rospy.Rate(5)

    ctrl_c = False
    def shutdownbot():
        rospy.loginfo("Shutdown the bot!")
        ctrl_c = True
        msg=Twist()
        msg.linear.x = 0
        msg.angular.z=0
        pub_= rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        pub_.publish(msg)  
    rospy.on_shutdown(shutdownbot)
    # rospy.spin()

    while not rospy.is_shutdown():
        global linear_speed,delta
        vel_msg = Twist(Vector3(linear_speed,0,0), Vector3(0,0,delta))
        # print("linear speed",linear_speed)
        # print("angular speed",delta)
        velocity_publisher.publish(vel_msg)
       
        rate.sleep()

if __name__ == '__main__':
    try:
        pd_controller()
    except rospy.ROSInterruptException: pass