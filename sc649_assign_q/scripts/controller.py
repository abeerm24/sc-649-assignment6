#!/usr/bin/env python3
'''
**************************************File to make the robot move in a circular path*********************************************
'''

import rospy
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry
import math

def main():
    rospy.init_node("controller",anonymous=True)
    pub = rospy.Publisher("/cmd_vel",Twist,queue_size=10)
    vel = Twist()
    vel.linear.x,vel.linear.y, vel.linear.z = 0.1,0,0
    #vel.angular.z = 0,0,0
    rate = rospy.Rate(10)

    while not (rospy.is_shutdown()):
        vel.linear.x = 0.5
        vel.angular.x,vel.angular.y,vel.angular.z = 0,0,0.5
        #print(vel)
        pub.publish(vel)
    rate.sleep()


if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass