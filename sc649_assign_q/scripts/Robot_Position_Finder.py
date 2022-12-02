#!/usr/bin/python3

import rospy
from sc649_assign_q.msg import Landmark
from sc649_assign_q.msg import Trilateration
from sc649_assign_q.msg import Robot_Position
import numpy as np
import time
import matplotlib.pyplot as plt

actual_x = 0
actual_y = 0
theta = 0 # We'll be using this variable to store the orientation since it's given that we know the orientation

Robot_Position_x = 0
Robot_Position_y = 0

state_x1 = 0
state_y1 = 0
state_r1 = 0
state_b1 = 0

state_x2 = 0
state_y2 = 0
state_r2 = 0
state_b2 = 0

state_x3 = 0
state_y3 = 0
state_r3 = 0
state_b3 = 0

thetaA = 0
thetaB = 0
thetaC = 0


def Trilateration_info(data):
	
	global state_x1 
	global state_y1 
	global state_r1 

	global state_x2  
	global state_y2 
	global state_r2 

	global state_x3 
	global state_y3 
	global state_r3 

	global theta
	global thetaA
	global thetaB
	global thetaC
	
	state_x1 = data.landmarkA.x
	state_y1 = data.landmarkA.y
	state_r1 = data.landmarkA.distance
	state_b1 = data.landmarkA.bearing

	state_x2 = data.landmarkB.x
	state_y2 = data.landmarkB.y
	state_r2 = data.landmarkB.distance
	state_b2 = data.landmarkB.bearing

	state_x3 = data.landmarkC.x
	state_y3 = data.landmarkC.y
	state_r3 = data.landmarkC.distance
	state_b3 = data.landmarkC.bearing

	# Given the bearing measurements and since the orientation of the robot theta is known, we can calculate the position estimate
	thetaA = np.pi/180*(theta + state_b1)  
	thetaB = np.pi/180*(theta + state_b2)
	thetaC = np.pi/180*(theta + state_b3)

def actualPosInfo(pose):
	global actual_x, actual_y, theta
	actual_x = pose.Robot_position_x
	actual_y = pose.Robot_position_y
	theta = pose.Robot_orientation


realX = []
realY = []
estX = []
estY = []

def programflow():

	rospy.init_node('Robot_Position_Finder')
	rospy.Subscriber('/measurement_data',Trilateration,Trilateration_info)
	rospy.Subscriber('robot_pose_data',Robot_Position,actualPosInfo)

	rate = rospy.Rate(5) #10 Hz
	count = 0

	while not rospy.is_shutdown():

		# A = (-2*state_x1 + 2*state_x2)
		# B = (-2*state_y1 + 2*state_y2)
		# C = (state_r1)**2 - (state_r2)**2 - (state_x1)**2 + (state_x2)**2 - (state_y1)**2 + (state_y2)**2 
		# D = (-2*state_x2 + 2*state_x3)
		# E = (-2*state_y2 + 2*state_y3)
		# F = (state_r2)**2 - (state_r3)**2 - (state_x2)**2 + (state_x3)**2 - (state_y2)**2 + (state_y3)**2
		# Robot_Position_x = (C*E - F*B)/(E*A - B*D + 0.000001)
		# Robot_Position_y = (C*D - A*F)/(B*D - A*E + 0.000001) 
		
		# Triangulation (with known orientation) implemented using landmarks A and B
		Robot_Position_x1 = (state_y2 - state_y1 + state_x1*np.tan(thetaA) - state_x2*np.tan(thetaB))/(np.tan(thetaA) - np.tan(thetaB))
		Robot_Position_y1 = (state_y2*np.tan(thetaA) - state_y1*np.tan(thetaB) + (state_x1 - state_x2)*np.tan(thetaA)*np.tan(thetaB))/(np.tan(thetaA) - np.tan(thetaB))
		
		# Triangulation (with known orientation) implemented using landmarks B and C
		Robot_Position_x2 = (state_y2 - state_y3 + state_x3*np.tan(thetaC) - state_x2*np.tan(thetaB))/(np.tan(thetaC) - np.tan(thetaB))
		Robot_Position_y2 = (state_y2*np.tan(thetaC) - state_y3*np.tan(thetaB) + (state_x3 - state_x2)*np.tan(thetaC)*np.tan(thetaB))/(np.tan(thetaC) - np.tan(thetaB))

		# Triangulation (with known orientation) implemented using landmarks A and C
		Robot_Position_x3 = (state_y1 - state_y3 + state_x3*np.tan(thetaC) - state_x1*np.tan(thetaA))/(np.tan(thetaC) - np.tan(thetaA))
		Robot_Position_y3 = (state_y1*np.tan(thetaC) - state_y3*np.tan(thetaA) + (state_x3 - state_x1)*np.tan(thetaC)*np.tan(thetaA))/(np.tan(thetaC) - np.tan(thetaA))

		Robot_Position_x,Robot_Position_y = (Robot_Position_x1 + Robot_Position_x2 + Robot_Position_x3)/3,(Robot_Position_y1 + Robot_Position_y2 + Robot_Position_y3)/3

		Robot_Pose_msg = Robot_Position(Robot_Position_x,Robot_Position_y,0)

		PosePublisher = rospy.Publisher('/robot_pose',Robot_Position,queue_size=5)
		PosePublisher.publish(Robot_Pose_msg)

		rospy.loginfo("Position_x: {:.2f}".format(Robot_Position_x))
		rospy.loginfo("Position_y: {:.2f}".format(Robot_Position_y))
		# rospy.loginfo("(thetaA): {: .2f}".format(thetaA))
		# rospy.loginfo("(thetaB): {: .2f}".format(thetaB))

		# Neglect the first 3 points in the trajectory as they are not part of the circle
		if count > 5:
			realX.append(actual_x)
			realY.append(actual_y)
			estX.append(Robot_Position_x)
			estY.append(Robot_Position_y)
		
		# Collect the next 80 points and then display them		
		if len(realX) > 80:
			mse = np.mean((np.array(realX) - np.array(estX))**2 + (np.array(realY) - np.array(estY))**2)
			print("Mean squared error: ", mse)
			plt.plot(realX,realY,"r",label="Real path")
			plt.plot(estX,estY,"b",label="estimated path")
			plt.legend()
			plt.show()
		rate.sleep()
		count+=1



if __name__ == '__main__':
	try:
		time.sleep(5)
		programflow()
	except rospy.ROSInterruptException:
		pass 
