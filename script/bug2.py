#!/usr/bin/env python
import rospy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

import math
import numpy as np

def callback1(data):
	global orient
	global position
	position=data.pose.pose.position
	orient=data.pose.pose.orientation

def callback(data):
	global front   #wall in front
	global left    #wall on left
	rarr=data.ranges
	cf=0
	cl=0
	j=100
	for j in range(100):
		if(rarr[120+j]<1):
			cf+=1
	for j in range(90):
		if(rarr[j]<1):
			cl+=1
	front=cf>1
	#print(front)
	left=cl>10

def implementation():
	global position
	global front
	global orient
	global position
	global rl

	p=rospy.Publisher("cmd_vel",Twist,queue_size=10)
	s=rospy.Subscriber("base_pose_ground_truth",Odometry,callback1)
	thres=0.9   #threshold value
	flag="target"
	rate=rospy.Rate(10)
	d=math.sqrt((9**2)+(4.5**2))
	reached=False   #flag for reached goal or not
	
	while(reached!=True):
		if(orient!=0):
			d_temp=math.sqrt((points[1,0]-position.x)**2 + (points[1,1]-position.y)**2)
			#print(d_temp)
			target_angle=math.atan((points[1,1]-position.y)/(points[1,0]-position.x)) - (2*np.arcsin(orient.z))
			#print(target_angle)
			A=points[0,:]
			B=points[1,:]
			C=np.array([position.x,position.y])
			area=abs(0.5*((A[0]-C[0])*(B[1]-A[1])-(A[0]-B[0])*(C[1]-A[1])))
			#print(area)
			rl=area<thres
			speed=Twist()

			if(d_temp<thres):    #reached the goal
				speed.linear.x=0
				speed.linear.y=0
				speed.linear.z=0
				reached=True    #setting the flag
				break
			else:
				if(front==True):   #obstacle in front
					speed.linear.x=0
				else:
					speed.linear.x=1
				omega=0     #defining the angular velocity
				if(flag=="target"):   #on the goal line
					if(rl==True):  #bot is on the line
						omega=min(target_angle,1)
					elif(front==True):   #obstacle in front
						omega=1
					else:
						omega=min(target_angle,1)
					speed.angular.z=omega

					if((front==True) or (left==True)):
						flag="wall"
				else:   #bot is away from the line so follow the wall
					if(front==True):      #wall in front
						omega=0.5
					else:
						if(left==True):   #wall on the left
							omega=0
						else:      #wall on the right
							omega=-1*0.7
					speed.angular.z=-1*omega

					if((rl==True) and front!=True):    #no wall in front and bot is on the goal line
						flag="target"
			p.publish(speed)
			rate.sleep()

if __name__ == '__main__':
	try:
		rospy.init_node('target',anonymous=True)
		orient=0
		points=np.array([[-8,-2],[4.5,9.0]])
		front=False
		left=False
		rl=False

		rospy.init_node('target',anonymous=True)
		rospy.Subscriber("base_scan",LaserScan,callback)
		implementation()

	except rospy.ROSInterruptException as e:
		pass