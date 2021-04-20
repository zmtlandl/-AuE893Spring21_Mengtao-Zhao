#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from move_robot import MoveTurtlebot3
# import apriltag
# from apriltag.python import apriltag
from pupil_apriltags import Detector

t1 = 0
kp = 0.08
kd = 0
ki = 0.0001
pre_error = 0
temp = 0

class LineFollower(object):

    def __init__(self):
    
        self.bridge_object = CvBridge()
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.moveTurtlebot3_object = MoveTurtlebot3()

    def camera_callback(self,data):
        cmd_vel_pub  = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        twist_object = Twist()

        # initial cmd_vel
        twist_object.linear.x   = 0
        twist_object.angular.z  = 0

        # We select bgr8 because its the OpneCV encoding by default
        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        height, width, channels = cv_image.shape

        # tss  = (width/1280)*2.1 
        # itss = int((width/1280)*2.5)
        # mll  = int(height/4) #Mask uper limit from bottom

        ## AR TAG DETECTION AND CONTROL

        height, width, channels = cv_image.shape
        # frame = cv_image[int(height/2):int(height)][1:int(width)]
        frame = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # April tag detector	
        # detector = apriltag.Detector()
        detector = Detector()
        
        result = detector.detect(frame)
        tag_detected = False

        cx = 0
        cy = 0
        if len(result)==1:
            tag_detected = True
            tag = result[0].tag_family
            cx  = result[0].center[0]
            cy  = result[0].center[1]
            corner_points = result[0].corners
            print(cx, cy)

        if len(result) > 1:
            print("Multiple tags detected")

        if tag_detected:
            print("Tag detected")

        global kp,kd,pre_error,temp    
        # Control
        err = 0
        if tag_detected:
            err = cx - width/2 +250
            if err >-5 and err <5:           #avoid steady state oscillation
                err = 0
            twist_object.angular.z = np.clip((-float(err)*kp/100 + kd*(-err+pre_error)/100),-0.2,0.2) 
            twist_object.linear.x  = 0.1
            pre_error = temp

            print("Mode: Tag following")
            # cv_image = cv2.putText(cv_image,'Mode: Tag detected',(int(20*tss),int(23*tss)),cv2.FONT_HERSHEY_SIMPLEX,0.5*tss,(0,0,255),1*itss, cv2.LINE_AA)	


        if tag_detected == False:
            twist_object.linear.x   = 0
            twist_object.angular.z  = 0

            print("Mode: Tag finding")
            # cv_image = cv2.putText(cv_image,'Mode: Tag finding',(int(20*tss),int(23*tss)),cv2.FONT_HERSHEY_SIMPLEX,0.5*tss,(0,0,255),1*itss, cv2.LINE_AA)

        # time
        global t1 
        t0 = float(rospy.Time.now().to_sec())
        timestep = (-t1+t0)
        t1 = t0  

        # Show image
        # cv_image = cv2.rectangle(cv_image,(int(10*tss),int(5*tss)),(int(290*tss),int(100*tss)),(0,250, 0),2*int(tss))

        if tag_detected:	
            tlx_point = int(corner_points[0,0])
            tly_point = int(corner_points[0,1])
            brx_point = int(corner_points[2,0])
            bry_point = int(corner_points[2,1])
            cv_image  = cv2.rectangle(cv_image,(tlx_point,tly_point),(brx_point,bry_point),(0,165,255),1*int(1000))
            side_length = np.sqrt(((tlx_point - brx_point)*(tlx_point - brx_point) +  (tly_point - bry_point)*(tly_point - bry_point))/2) 
            z_distance= (75*160/side_length)*75/44    #(160/270)

            msg5     = str("Z distance is " + str(int(z_distance)) + "cm")
            # cv_image = cv2.putText(cv_image,msg5,(int(tlx_point-side_length),int(tly_point-20)),cv2.FONT_HERSHEY_SIMPLEX,0.4*tss,(0,0,255),1*itss, cv2.LINE_AA)
            if z_distance < 15:
                twist_object.linear.x   = np.clip((float(z_distance-38)/100),-0.1,0.1)
                if z_distance < 5:
                    twist_object.linear.x = 0
                twist_object.angular.z  = 0
                print(" Too close to tag, stopped turtlebot")
                # cv_image = cv2.putText(cv_image,'Too close to tag',(int(20*tss),int(120*tss)),cv2.FONT_HERSHEY_SIMPLEX,0.5*tss,(0,0,255),1*itss, cv2.LINE_AA)
            
        # msg1     = str("Lane  error            " + str(int(err*100/width))+" %")
        # cv_image = cv2.putText(cv_image,msg1,(int(20*tss),int(45*tss)),cv2.FONT_HERSHEY_SIMPLEX,0.4*tss,(0,255,0),1*itss, cv2.LINE_AA)

        # msg2     = str("Linear  velocity        " + str(float(int(1000*twist_object.linear.x))/1000))
        # cv_image = cv2.putText(cv_image,msg2,(int(20*tss),int(60*tss)),cv2.FONT_HERSHEY_SIMPLEX,0.4*tss,(0,255,0),1*itss, cv2.LINE_AA)

        # msg3     = str("Angular velocity        " + str(float(int(1000*twist_object.angular.z))/1000))
        # cv_image = cv2.putText(cv_image,msg3,(int(20*tss),int(75*tss)),cv2.FONT_HERSHEY_SIMPLEX,0.4*tss,(0,255,0),1*itss, cv2.LINE_AA)

        # msg4     = str("Update time (ms)     " + str((int(1000*timestep))))
        # cv_image = cv2.putText(cv_image,msg4,(int(20*tss),int(90*tss)),cv2.FONT_HERSHEY_SIMPLEX,0.4*tss,(0,255,0),1*itss, cv2.LINE_AA)

        # Print
        # print("\n       Angular value sent=> "+str(float(int(1000*twist_object.angular.z))/1000))
        # print("       Linear  value sent=> "+str(float(int(1000*twist_object.linear.x))/1000))
        # print("          Lane error     => "+str(int(err*100/width))+" %")
        # print("             cx          => "+str(float(int(100*cx))/100)+"\n             cy          => "+str(float(int(100*cy))/100))
        # print("        Timestep (sec)   => "+str(float(int(1000*timestep))/1000))
        # print("         Update rate     => "+str(int(1/timestep)))
        if tag_detected:	
            print(corner_points)
            print(" Tag side length = ", side_length) 
            print(" Tag z disatnce  = ",  z_distance)		
            
        print(" \n")

        cv2.namedWindow("Apriltag follower",cv2.WINDOW_NORMAL)
        cv2.imshow("Apriltag follower", cv_image)
        cv2.resizeWindow("Apriltag follower", (500,500))
        cv2.waitKey(1)

        # Debug
        #twist_object.linear.x   = 0
        #twist_object.angular.z  = 0	

        # Make it start turning  
        self.moveTurtlebot3_object.move_robot(twist_object)
        print("\n")
        
    def clean_up(self):
        self.moveTurtlebot3_object.clean_class()
        cv2.destroyAllWindows()
        
def main():
    rospy.init_node('line_following_node', anonymous=True)
    
    line_follower_object = LineFollower()
    
    rate = rospy.Rate(1)
    # ctrl_c = False
    # def shutdownhook():
    #     # works better than the rospy.is_shut_down()
    #     line_follower_object.clean_up()
    #     rospy.loginfo("shutdown time!")
    #     ctrl_c = True
    
    # rospy.on_shutdown(shutdownhook)
    
    while not rospy.is_shutdown():
        rate.sleep()
        

if __name__ == '__main__':
    main()
