#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from numpy import inf
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg  import Twist, Vector3
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int64
from termcolor import colored
from move_robot import MoveTurtlebot3
import random

distancefromwall     = 0.20
x    = np.zeros((360))# lidar scan data
f_d  = 0              # front wall distance
r_d  = 0              # right wall distance
fr_d = 0
prev_err = 0
mission_stage = 0
front_wall = 0

delta    = 0
f_l_dist = 0
f_r_dist = 0 
l_dist   = 0
r_dist   = 0
current_tag   = 0

kp = 0.08
kd = 0
ki = 0.0001
pre_error = 0
err_sum = 0
temp = 0
t1 = 0
first_line_confirmation = False
line_find_factor = 0
invoke_line_finder = False
n = 0
a_temp = 0
b_temp = 0


def callback(data):
    global x,f_d,r_d,fr_d,front_wall
    global f_l_dist,f_r_dist,l_dist,r_dist

    x  = list(data.ranges)
    for i in range(360):
        if x[i] == inf:
            x[i] = 7
        if x[i] == 0:
            x[i] = 6

        # store scan data 
    r_d  = min(x[270:-15])              # right wall distance
    f_d  = min(min(x[0:15],x[-15:])) # front wall distance
    fr_d = min(x[-15:])              # front right distance

    
    #x  = list(data.ranges)
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
    
    front_wall = min(min(x[:20]),min(x[-20:]))

def camera_callback(data):
    # We select bgr8 because its the OpneCV encoding by default
    bridge_object = CvBridge()
    cv_image = bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
    
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
    global invoke_line_finder

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

    global kp,ki,kd,pre_error,err_sum,temp
    err = cx - width/2 +300

    #avoid steady state oscillation
    if err > -5 and err < 5:
        err = 0
    
    global a_temp, b_temp
    # line following maneuver control/// why here use - err
    angular_zvel = np.clip((-float(err)*kp/100 + kd*(-err+pre_error)/100),-0.2,0.2)
    a_temp                 = np.clip((-float(err)*kp/100 + kd*(-err+pre_error)/100),-0.2,0.2)

    linear_vel = 0.05 #np.clip(0.2*(1-abs(a_temp)/0.2),0,0.2)
    b_temp                = 0.05 #np.clip(0.2*(1-abs(a_temp)/0.2),0,0.2)


    # after each control
    pre_error = temp
    err_sum = err_sum + err
    
def tmnt_controller():

    #Setup
    global x,r_d,f_d,fr_d,prev_err,front_wall
    global distancefromwall
    global first_line_confirmation
    global t1,n
    global line_find_factor
    global kp,ki,kd,pre_error,err_sum,temp
    global a_temp, b_temp
    global invoke_line_finder
    

    rospy.init_node('auefinals', anonymous=True)
    velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=3)
    scan_subscriber    = rospy.Subscriber('/scan', LaserScan, callback)
    rate               = rospy.Rate(20)   
    bridge_object = CvBridge()
    image_sub = rospy.Subscriber("/raspicam_node/image_raw",LaserScan,camera_callback)          

    while not rospy.is_shutdown():
        
        print ('front wall :', front_wall)
        
        if front_wall > 0.4 and front_wall < 7:
            print('######################  WALL FOLLOWING  #########################')
            delta = distancefromwall-r_d   # distance error 
            #PD controller
            PID_output  = delta*0.6 + (delta-prev_err)*40
            prev_err    = delta

            #clip PID output
            angular_zvel = np.clip(PID_output,-1.0,1.0)

            vel_factor   = np.clip((1-abs(delta)/1.2),0,1)
            linear_vel   = 0.09   #0.15

            if f_d < 0.75 and fr_d < 1:
                angular_zvel = 0.06               #0.5
                print('turn')

            if linear_vel < 0:
                angular_zvel = -1*angular_zvel

            if r_d ==0:
                angular_zvel = 0
            
            #check IOs
            print('distance from right wall in cm =',format(int(r_d*100)),'/',format(distancefromwall*100))
            print('distance from front wall in cm =',format(f_d*100))
            print('linear_vel=',format(linear_vel),' angular_vel=',format(angular_zvel))
            rospy.loginfo(r_d)
            rospy.loginfo('\n') 
        
        elif front_wall < 0.4:
            print('######################  OBSTRACLE AVOIDANCE  #########################')
            Head_on_threshold        = 1.2
            side_obstacle_confidence = 1
            #print(x)
    
            delta = l_dist - r_dist
        
            if min(f_l_dist, f_r_dist) < 3.5: # obstacle ahread
                print ("f_l_dist: %.2f f_r_dist: %.2f delta: %.2f" %(f_l_dist,f_r_dist,delta))
                approach  = f_l_dist - f_r_dist
            
                if approach > 1: # Obstacle on the right
                    delta = min(3.5,delta + 1/f_r_dist)
                elif approach < -1: #Obstacle on the left
                    delta = max(-3.5, delta - 1/f_l_dist)
                else: #Head-on
                
                    # delta = random.choice([min(3.5, fwd_wt/f_l_dist), max(-3.5,-1 * fwd_wt/f_l_dist)]) #f_l_dist is the same as f_r_dist
                    #Don't over react if the head on collision is not imminent

                    if min(f_l_dist,f_r_dist)<Head_on_threshold: #imminent collision
                        print ("Alert!!!!")	
                        if -1*side_obstacle_confidence<delta<side_obstacle_confidence: #No danger of side collision
                            print("Delta ignored")
                            if f_l_dist < f_r_dist:
                                delta = -1.5
                            elif f_l_dist > f_r_dist:
                                delta = +1.5
                            else:
                                delta = random.choice([-1.5,1.5]) 
                        else:
                            delta = np.sign(delta) * 1.0 #side collision danger
            angular_zvel  = delta
            linear_vel   = 0.06

        elif front_wall == 7:
            print('######################  LINE FOLLOW  #########################')

             # control to find lane
            t0 = float(rospy.Time.now().to_sec())
            timestep = (t0-t1)
            linear_vel = 0
            angular_zvel = -0.06
            # print(colored("invoke_lane_finder:",'red'),invoke_line_finder)

            '''if invoke_line_finder:
                if first_line_confirmation == False:
                    print("Mode - Line finding maneuver")
                    cv_image2 = cv2.putText(cv_image2,"Mode: Line following maneuver-1", (20,24),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.CV_AA)

                linear_vel = 0.08
                angular_zvel = 0.0

                if first_line_confirmation:
                    linear_vel = np.clip((line_find_factor * 0.05),0,0.08)
                    angular_zvel = 0.06
                    n = n + timestep*angular_zvel

                if n > 3.14159*1.5 and invoke_line_finder:
                    n = 0
                    line_find_factor = line_find_factor + 1

                print("Mode - Line finding maneuver - 2")
                # cv_image2 = cv2.putText(cv_image2,"Mode: Line following maneuver-2", (20,24),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
                print("Angle turned by bot=>"+str(n))
            t1 = t0
            if a_temp == angular_zvel and b_temp == linear_vel:
                line_find_factor = 0
                n = 0'''

        #publish cmd_vel
        vel_msg = Twist(Vector3(linear_vel,0,0), Vector3(0,0,angular_zvel))
        # sub_status = rospy.Subscriber("/mission_stage", Int64, callback_mission_stage)
        if mission_stage ==2:
            vel_msg = Twist(Vector3(0,0,0), Vector3(0,0,0))
        velocity_publisher.publish(vel_msg)
        rate.sleep()
        if mission_stage==2:
            break
        
    velocity_publisher.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
    rospy.loginfo('Turtlebot stopped')

def callback_mission_stage(msg):
    global mission_stage
    mission_stage =  msg.data


def callback_current_tag(msg):
    global current_tag
    current_tag =  msg.data 


if __name__ == '__main__':
    try:
        #start turtllebot
        tmnt_controller()
    except rospy.ROSInterruptException: pass
