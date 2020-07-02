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
rob_x = 0.0
rob_y = 0.0
rob_theta = 0.0
tag_store = {} # store tag id and (x,z) coordinates
num_tags = 10
tag_detect = False

check_right = True


# getting the current robot pose
def robotPose(msg):
    global rob_x
    global rob_y
    global rob_theta

    rob_x = msg.pose.pose.position.x 
    rob_y = msg.pose.pose.position.y 

    rot_q = msg.pose.pose.orientation
    (_,_,rob_theta) = euler_from_quaternion([rot_q.x,rot_q.y,rot_q.z,rot_q.w])

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

        # for x in range(0,20):
        #     robotVel.publish(turn_right)
        #     rate.sleep()
        #     if tag_detect:
        #         break

        # if tag_store[tag_id][0] == 0:
        #     forward.linear.x = 0.0                        

    else:
        check_right = True
        rospy.loginfo('Detected tag {}'.format(tag_store[tag_id][0])) # read tag
        error_distance = (tag_store[tag_id][2])   # we take error between tag and robot
        offset = (tag_store[tag_id][1]) # this offset
        angle2goal = (atan2(offset,error_distance)) # arctangent function

        rospy.loginfo("error_distance: {}".format(error_distance)) 
        rospy.loginfo("offset: {}".format(offset)) 
        #rospy.loginfo("angle2goal: {}".format(angle2goal)) 

        #rospy.loginfo("rob_x: {}".format(rob_x)) 
        # rospy.loginfo("rob_y: {}".format(rob_y)) 
        # rospy.loginfo("rob_theta: {}".format(rob_theta)) 
        
        # cases :
        if offset > 0.12:  
            # if (angle2goal) > 0.1: 
                # forward.angular.z = 0.1         # we turn    
            rospy.loginfo("right")

            robotVel.publish(turn_right)
            rate.sleep() 
            # robotVel.publish(forward)                

        elif offset <0.04:  
            # if (-angle2goal) > 0.1: 
                # forward.angular.z = 0.1         # we turn    
            rospy.loginfo("left")

            robotVel.publish(turn_left)
            rate.sleep()
            # robotVel.publish(forward) 

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


        # forward.linear.x = 0.0


        # turn.angular.z = radians(0)


forward = Twist()
forward.linear.x = 0.1

turn_right = Twist()
turn_right.angular.z = radians(-3)

turn_big_right = Twist()
turn_big_right.angular.z = radians(-10)

turn_left = Twist()
#turn.linear.x = 0.0
turn_left.angular.z = radians(3)

turn_big_left = Twist()
#turn.linear.x = 0.0
turn_big_left.angular.z = radians(10)
                  


if __name__ == '__main__':

    rospy.init_node("BOT",anonymous=True)
    rate = rospy.Rate(50)

    robotVel = rospy.Publisher('/RosAria/cmd_vel',Twist,queue_size=10)
    subPose = rospy.Subscriber('/RosAria/pose',Odometry,robotPose)
    tagdetectSub = rospy.Subscriber('/tag_detections',AprilTagDetectionArray,getTagPosition) 

    # follow_waypoints()
    # robotVel.publish(forward)

    # rob_x,rob_y,rob_theta # working
    # tag_detect # working
    
    #rospy.loginfo('Following Waypoint!')
    #rospy.loginfo(tag_store)

    # if tag_detect:
    #     rospy.loginfo('Detected tag {}'.format(tag_store[tag_id])) # read tag
    #     error_distance = tag_store[tag_z] - rob_x   # we take error between tag and robot
    #     offset = tag_store[tag_x] - rob_y # this offset
    #     angle2goal = atan2(offset,error_distance) # arctangent function
        
    #     # cases :
    #     while error_distance > 0.1:  
    #         # forward.linear.x = 0.1 # if tag is in same line we go forward
    #         robotVel.publish(forward)
    #         rate.sleep()

    #     while offset > 0:  
    #         if (angle2goal - rob_theta) > 0: 
    #             # forward.angular.z = 0.1         # we turn    
    #             robotVel.publish(turn)
    #             rate.sleep()         

    # else:
    #     rospy.loginfo('TAG OUT OF RANGE')   

    rospy.spin()





    





