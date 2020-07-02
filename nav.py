#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point,Twist
from math import atan2
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

x = 0.0
y = 0.0
theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (_,_,theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

rospy.init_node("speed_controller")

sub = rospy.Subscriber("/RosAria/pose",Odometry,newOdom)
pub = rospy.Publisher("/RosAria/cmd_vel",Twist,queue_size=1)

speed = Twist()
rate = rospy.Rate(10)

goal = Point()
goal.x = 5
goal.y = 5

while not rospy.is_shutdown():
    inc_x = goal.x - x
    inc_y = goal.y - y

    angle_goal = atan2(inc_y,inc_x)

    if abs(angle_goal-theta) > 0.1:
        speed.linear.x = 0.0
        speed.angular.z = 0.3

    else:
        speed.linear.x = 0.3
        speed.angular.z = 0.0

    pub.publish(speed)
    rate.sleep()
    rospy.loginfo("Current position of x is %s",x)
    rospy.loginfo("Current position of y is %s",y)    


# 