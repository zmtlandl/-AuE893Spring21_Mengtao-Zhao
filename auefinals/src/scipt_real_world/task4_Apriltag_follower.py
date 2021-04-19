#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from apriltag_ros.msg import AprilTagDetectionArray
from move_robot import MoveTurtlebot3

rotate = 0
forward = 0


class Apriltag_follower(object):

    def __init__(self):

        self.bridge_object = CvBridge()
        self.publish = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.image_sub = rospy.Subscriber('/tag_detections_image', Image, self.camera_callback)
        self.sub = rospy.Subscriber('/tag_detections', AprilTagDetectionArray, self.callback)

    def camera_callback(self, data):

        cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        cv2.imshow("Scan", cv_image)
        cv2.waitKey(1)

    def callback(self, data):
        global rotate, forward
        try:
            rotate = data.detections[0].pose.pose.pose.position.x - 0.089
            forward = data.detections[0].pose.pose.pose.position.z
            twist_object = Twist()
            Kp_forward = 0.5
            Kp_rotate = 6
            count = 0
            if forward < 0.1:
                twist_object.linear.x = 0
            elif forward == 10000:
                twist_object.linear.x = 0
            else:
                twist_object.linear.x = forward * Kp_forward
                if rotate < 0:
                    count = 1
                if rotate > 0:
                    count = -1
                twist_object.angular.z = count * Kp_rotate * abs(rotate) / 1.9

            self.publish.publish(twist_object)
        except IndexError:
            twist_object = Twist()
            twist_object.linear.x = 0
            rospy.loginfo('No tag detected')
            self.publish.publish(twist_object)


def main():
    rospy.init_node('april_tag_node', anonymous=True)

    Apriltag_follower()

    rate = rospy.Rate(5)

    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()
