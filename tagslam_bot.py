#!/usr/bin/env python


import rospy
from geometry_msgs.msg import Twist
from apriltag_ros.msg import AprilTagDetectionArray
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
from math import atan2,radians

# initializing to zero
tag_x = 0.0
tag_z = 0.0
tag_store = {} # store tag id and (x,z) coordinates
tag_detect = False

check_right = True



# getting the values from apriltag_ros/AprilTagDetectionArray topic
def getTagPosition(msg):
    global tag_detect
    global check_right
    try:
        tag_x = msg.detections[0].pose.pose.pose.position.x # x position
        tag_z = msg.detections[0].pose.pose.pose.position.z # z position
        tag_id = msg.detections[0].id[0] # get tag id      
        rate.sleep()                  

        tag_store[tag_id] = (tag_id,tag_x,tag_z) # we take the x and z value and store in array
        tag_detect = True


    except:
        rospy.loginfo("TAG not found!") 
        if check_right:    
            for x in range(10):
                robotVel.publish(turn_big_left)
                if tag_detect:
                    break
                rate.sleep()
            check_right = False
        else: 
            for x in range(5):
                robotVel.publish(turn_big_right)
                if tag_detect:
                    break    
                rate.sleep() 
            check_right = True
                     

    else:
        check_right = True
        rospy.loginfo('Detected tag {}'.format(tag_store[tag_id][0])) # read tag
        error_distance = (tag_store[tag_id][2])   # we take error between tag and robot
        offset = (tag_store[tag_id][1]) # this offset

        rospy.loginfo("error_distance: {}".format(error_distance)) 
        rospy.loginfo("offset: {}".format(offset)) 

        
        # cases :
        if offset > 0.12:    
            rospy.loginfo("right")

            robotVel.publish(turn_right)
            rate.sleep()              

        elif offset <0.04:  
            rospy.loginfo("left")

            robotVel.publish(turn_left)
            rate.sleep()

        elif error_distance > 0.55:  
            rospy.loginfo('forward!')
            forward.linear.x = 0.15 # if tag is in same line we go forward
            robotVel.publish(forward)
            rate.sleep()

        elif error_distance < 0.06:  
            rospy.loginfo('forward!')
            forward.linear.x = 0.15 # if tag is in same line we go forward
            robotVel.publish(forward)
            rate.sleep()                              


forward = Twist()
forward.linear.x = 0.1

turn_right = Twist()
turn_right.angular.z = radians(-3)

turn_big_right = Twist()
turn_big_right.angular.z = radians(-10)

turn_left = Twist()
turn_left.angular.z = radians(3)

turn_big_left = Twist()
turn_big_left.angular.z = radians(10)
                  


if __name__ == '__main__':

    rospy.init_node("BOT",anonymous=True)
    rate = rospy.Rate(50)

    robotVel = rospy.Publisher('/RosAria/cmd_vel',Twist,queue_size=10)
    tagdetectSub = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,getTagPosition) 

    rospy.spin()





    





