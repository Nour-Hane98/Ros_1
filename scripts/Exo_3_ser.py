#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from odometry.srv import moveN

x = 0.0

def get_position(msg):
    global x
    x = msg.pose.pose.position.x

def move(req):
    rospy.loginfo("Request received: move %.2f meters", req.a)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    r = rospy.Rate(10)
    vel = Twist()
    
    rospy.wait_for_message('/odom', Odometry)

    # Control proporcional
    kp = 0.4
    x_target = x + req.a
    tol = 0.01

    time_limit = 10
    time_ini = rospy.Time.now().to_sec()

    while abs(x_target - x) > tol and (rospy.Time.now().to_sec() - time_ini) < time_limit:
        vel.linear.x = kp * (x_target - x)
        pub.publish(vel)
        r.sleep()

    vel.linear.x = 0
    pub.publish(vel)

    return True  

def move_robot_server():
    rospy.init_node('move_robot_server')
    rospy.Subscriber('/odom', Odometry, get_position)
    s = rospy.Service('move_robot', moveN, move)
    rospy.loginfo("The server is ready and waiting for moting request..")
    rospy.spin()

if __name__ == "__main__":
    move_robot_server()
