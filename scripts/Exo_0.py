#!/usr/bin/env python3

import sys
import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

yaw = 0

def get_pos (msg): 
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation 
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w] 
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list) 
    
if __name__ == '__main__': 
    
    rospy.init_node('robot_rotation') 
    sub = rospy.Subscriber ('/odom', Odometry, get_pos) 
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
    r = rospy.Rate(10)

    angle_deg = 90
    kp = 0.5
    rospy.wait_for_message('/odom',Odometry)
    

    yaw_inicial = yaw
    kp = 0.5
    target = yaw_inicial + math.radians(angle_deg)
    vel = Twist()

    
    
    while not rospy.is_shutdown():
        error = target - yaw

        if abs(error) < 0.01:
           vel.angular.z = 0.0
           pub.publish(vel)
           break


        vel.angular.z = kp * error 
        pub.publish(vel)
        r.sleep()
        
