#!/usr/bin/env python

import rospy
import math
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist,Point


x = y = theta = 0.0

def newOdom(msg):
    global x
    global y
    global theta

    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y

    rot_q = msg.pose.pose.orientation
    (_,_,theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

def curve():
    pass


if __name__ == '__main__':
    rospy.init_node('Curve')

    rospy.Subscriber('RosAria/pose',Odometry,newOdom)
    rospy.Publisher('RosAria/cmd_vel',Twist,queue_size=10)