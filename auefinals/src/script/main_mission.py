#!/usr/bin/env python

import roslaunch
import rospy
from geometry_msgs.msg  import Twist, Vector3
import numpy as np
import cv2

# initiate main node
rospy.init_node('aue_final_team3', anonymous=True)
uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
roslaunch.configure_logging(uuid)

# wait for 'launch world'
rospy.sleep(5)

# launch SLAM mapping
launch_slam = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mengtao/catkin_ws/src/turtlebot3/turtlebot3_slam/launch/turtlebot3_slam.launch"])
launch_slam.start()
rospy.loginfo("slam initiated")
rospy.sleep(2)

# mission status callback
mission_stage = 1
   
pub_vel    = rospy.Publisher('/cmd_vel', Twist, queue_size=3)

# wall follower
launch_wf = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mengtao/catkin_ws/src/auefinals/src/launch/turtlebot3_autonomy_final_wall_follower.launch"])

if mission_stage ==1:
    launch_wf.start()
    rospy.loginfo("wall following initiated")

cv2.imshow("Wall follow mode",np.zeros([1,1],dtype=np.uint8))
while mission_stage==1:
    rospy.loginfo('mode - wall following')
    rospy.sleep(0.3)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('1'):
        mission_stage = 2
cv2.destroyAllWindows()  
pub_vel.publish(Twist(Vector3(0,0,0), Vector3(0,0,0))) 
launch_wf.shutdown()


 

# obstacle avoidance
launch_oa = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mengtao/catkin_ws/src/auefinals/src/launch/turtlebot3_autonomy_final_obstacle_avoidance.launch"])

if mission_stage ==2:
    launch_oa.start()
    print(' starting obstacle avoidance mode')

cv2.imshow("Obstacle avoidance model",np.zeros([1,1],dtype=np.uint8))
while mission_stage==2:
    rospy.loginfo('obstacle avoidance mode')
    rospy.sleep(0.2)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('2'):
        mission_stage = 3

cv2.destroyAllWindows()  
pub_vel.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
launch_oa.shutdown()

# line follower
launch_lf = roslaunch.parent.ROSLaunchParent(uuid, ["/home/mengtao/catkin_ws/src/auefinals/src/launch/turtlebot3_autonomy_final_line_follower.launch"])

if mission_stage ==3:
    launch_lf.start()
    print(' starting line follower mode')

cv2.imshow("line follower model",np.zeros([1,1],dtype=np.uint8))
while mission_stage==3:
    rospy.loginfo('line follower mode')
    rospy.sleep(0.2)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('3'):
        mission_stage = 4

cv2.destroyAllWindows()  
pub_vel.publish(Twist(Vector3(0,0,0), Vector3(0,0,0)))
launch_lf.shutdown()


rospy.loginfo('closing all nodes')
rospy.sleep(5)



    


