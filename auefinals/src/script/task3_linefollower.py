#!/usr/bin/env python3
import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import Image
from termcolor import colored
from move_robot import MoveTurtlebot3

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
class LineFollower(object):
    def __init__(self):
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        # self.moveTurtlebot3_object = MoveTurtlebot3()

    def camera_callback(self,data):
        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        
        height, width, channels = cv_image.shape
        # print("height:",height,"width:",width)
        crop_img = cv_image[int(height/2)+160:int(height)][1:int(width)]
        # print("crop_img:",crop_img)

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

        global kp,ki,kd,pre_error,err_sum,temp
        err = cx - width/2

        #define the nodes
        cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        twist_object = Twist()

        #avoid steady state oscillation
        if err > -5 and err < 5:
            err = 0
        
        # line following maneuver control/// why here use - err
        twist_object.angular.z = np.clip(-float(err)*kp/100 + kd*(-err+pre_error)/100,-0.2,0.2)
        a_temp = np.clip((-float(err)*kp/100 + kd*(-err+pre_error)/100),-0.2,0.2)

        twist_object.linear.x = np.clip(0.2*(1-abs(a_temp)/0.2),0,0.2)
        b_temp = np.clip(0.2*(1-abs(a_temp)/0.2),0,0.2)


        # after each control
        pre_error = temp
        err_sum = err_sum + err

    
        # control to find lane
        t0 = float(rospy.Time.now().to_sec())
        timestep = (t0-t1)
        # print(colored("invoke_lane_finder:",'red'),invoke_line_finder)

        if invoke_line_finder:
            if first_line_confirmation == False:
                print("Mode - Line finding maneuver")
                cv_image2 = cv2.putText(cv_image2,"Mode: Line following maneuver-1", (20,24),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)

                twist_object.linear.x = 0.1
                twist_object.angular.z = 0.0

            if first_line_confirmation:
                twist_object.linear.x = np.clip((line_find_factor * 0.05),0,0.08)
                twist_object.angular.z = 0.2
                n = n + timestep*twist_object.angular.z

                if n > 3.14159*1.5 and invoke_line_finder:
                    n = 0
                    line_find_factor = line_find_factor + 1

                print("Mode - Line finding maneuver - 2")
                # cv_image2 = cv2.putText(cv_image2,"Mode: Line following maneuver-2", (20,24),cv2.FONT_HERSHEY_SIMPLEX,0.5,(255,0,0),1,cv2.LINE_AA)
                print("Angle turned by bot=>"+str(n))
        t1 = t0
        if a_temp == twist_object.angular.z and b_temp == twist_object.linear.x:
            line_find_factor = 0
            n = 0
        cmd_vel_pub.publish(twist_object)

        # Display
        # cv_image2 = cv2.rectangle(cv_image2,(10,5),(290,100),(0,255,0),2)
        # msg1 = str("Line error " + str(int(err*100/width)) + " %")
        # cv2.putText(cv_image2, msg1, (20,45),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0),1,cv2.LINE_AA)
        
        # msg2 = str("Line velocity " + str(float(int(1000*twist_object.linear.x))/1000))
        # cv2.putText(cv_image2, msg2, (20,60),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0),1,cv2.LINE_AA)
        
        # msg3 = str("Angular velocity " + str(float(int(1000*twist_object.angular.z))/1000) )
        # cv2.putText(cv_image2, msg3, (20,75),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0),1,cv2.LINE_AA)
        
        # msg4 = str("Update time (ms) " + str((int(1000*timestep))))
        # cv2.putText(cv_image2, msg4, (20,90),cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,255,0),1,cv2.LINE_AA)

        cv2.namedWindow("Line Follower",cv2.WINDOW_NORMAL)
        cv2.imshow("Line Follower",cv_image)
        cv2.waitKey(1)

        # rospy.loginfo("Angular Value Sent ===>" + str(twist_object.angular.z))
        # self.moveTurtlebot3_object.move_robot(twist_object)

    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()

def main():
    
    rospy.init_node('line_following_node', anonymous=True)
    line_follower_object = LineFollower()
    rate = rospy.Rate(10)
    ctrl_c = False

    
    while not rospy.is_shutdown():
        rate.sleep()
    cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    cmd_vel_pub.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: pass