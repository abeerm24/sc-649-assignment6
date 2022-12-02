#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import math
import numpy as np
import time

from sc649_assign_q.msg import Measurement, Landmark, Robot_Position



## quaternion to euler
def quat2euler(x,y,z,w):
    quat = [x,y,z,w]
    return euler_from_quaternion(quat)
########################


## Globals
pose = [0.0, 0.0, 0.0]

landmarkA = [ -5,  7]
varA = 0.20
varAbearing = 0.5
landmarkB = [  5,  7]
varB = 0.15
varBbearing = 0.5
landmarkC = [  0, -7] 
varC = 0.10
varCbearing = 0.5
########################


## Odometry callback
def callback(data):
    global pose
    x  = data.pose.pose.orientation.x;
    y  = data.pose.pose.orientation.y;
    z = data.pose.pose.orientation.z;
    w = data.pose.pose.orientation.w;
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, (180/np.pi)*quat2euler(x,y,z,w)[0]]
########################


## Euclidean Distance
def dist(p1, p2):
    return ( (p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 )**(0.5)
########################

def bearing(p1, p2):
    rb = (180/np.pi)*(math.atan2(p2[1]-p1[1],p2[0]-p1[0]) - p1[2])

    if(rb >= 180):
        rb = rb - 360
    if(rb < -180):
        rb = rb + 360

    return ( rb )
######################## 

## Node 
def trilateration_pub():
    global landmarkA, landmarkB, landmarkC, varA, varB, varC, pose
    
    rospy.init_node('Problem_Setup_Node', anonymous=True)
    rospy.Subscriber('/odom', Odometry, callback)
    
    pub = rospy.Publisher('measurement_data', Measurement, queue_size=10)
    pub_pose = rospy.Publisher('robot_pose_data', Robot_Position, queue_size=10)
    rate = rospy.Rate(8)
    while not rospy.is_shutdown():
        lA = Landmark(landmarkA[0], landmarkA[1], dist(pose, landmarkA)+np.random.normal(0,varA), bearing(pose,landmarkA)+np.random.normal(0,varAbearing),varA,varAbearing)
        lB = Landmark(landmarkB[0], landmarkB[1], dist(pose, landmarkB)+np.random.normal(0,varB), bearing(pose,landmarkB)+np.random.normal(0,varBbearing),varB,varBbearing)
        lC = Landmark(landmarkC[0], landmarkC[1], dist(pose, landmarkC)+np.random.normal(0,varC), bearing(pose,landmarkC)+np.random.normal(0,varCbearing),varC,varCbearing)
        t = Measurement(lA, lB, lC)
        p = Robot_Position(pose[0],pose[1],pose[2])
        #rospy.loginfo("Sent :\n{}".format(t))
        pub.publish(t)
        pub_pose.publish(p)
        rate.sleep()
########################

if __name__ == '__main__':
    try:
        time.sleep(5)
        trilateration_pub()
    except rospy.ROSInterruptException:
        pass
