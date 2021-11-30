#!/usr/bin/env python

import rospy
import random
import numpy as np

from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point

import math

def ransac(x,y,n,n_cycles,p_thres,thres):   #n is number of iteration, thres is threshold value, p_thres below which lines can't be drawn
	global xarr
	global yarr
	
	x=ranges[:,0]
	y=ranges[:,1]

	ind=[]
	for i in xrange(361):
		ind.append(i)
	xarr=[-1]
	yarr=[-1]

	i=0
	for _ in xrange(n_cycles):
		inl=[]
		oul=[]

		for i in xrange(n):
			temp_inl=[]
			temp_oul=[]
			ind1=random.randint(0,len(ind)-1)
			ind2=random.randint(0,len(ind)-1)
			if(ind1==ind2):
				continue

			x1=x[ind[ind1]]
			y1=y[ind[ind1]]

			x2=x[ind[ind2]]
			y2=y[ind[ind2]]

			if(x1!=0 and x2!=0):
				for j in ind:
					x0=x[j]
					y0=y[j]
					if(x0!=0 or y0!=0):
						d=(abs((y2-y1) * x0 - (x2-x1) * y0 + x2 * y1 - y2 * x1))/(math.sqrt((y2 - y1) **2 + (x2 - x1) **2))
						if(d<thres):
							temp_inl.append(j)
						else:
							temp_oul.append(j)
			if(len(inl)<len(temp_inl)):
				inl=temp_inl
				oul=temp_oul
				mx=np.max(ranges[inl,0])   #max of x
				ex=np.min(ranges[inl,0])   #end of x
				end_p=ranges[inl,:]
				my=end_p[np.argmax(end_p[:,0]),1]
				ey=end_p[np.argmin(end_p[:,0]),1]
				in1=ind1
				in2=ind2

		if(len(inl)>p_thres):
			xarr.append(mx)
			yarr.append(my)
			xarr.append(ex)
			yarr.append(ey)

			ind=oul

def visaul():
	global ranges
	rate=rospy.Rate(10)
	p=rospy.Publisher("ransac_vis",Marker,queue_size=10)
	n=50   #number of iterations
	thres=0.05
	p_thres=10
	x=y=1
	n_cycles=10
	while not rospy.is_shutdown():
		try:
			ransac(x,y,n,n_cycles,p_thres,thres)
			m=Marker()
			#m1=Marker()
			m.header.frame_id="/base_link"
			# m.header.stamp=rospy.Time.now()
			m.ns="ransac"
			m.id=0
			m.type=Marker.LINE_LIST
			m.action=Marker.ADD
			m.scale.x=0.1
			#m.scale.y=0.1
			m.color.g=1.0
			m.color.r=0.0
			m.color.b=0.0
			m.color.a=1.0
			m.pose.position.x=1
			m.pose.position.y=1
			m.pose.position.z=1
			m.pose.orientation.x=0.0
			m.pose.orientation.y=0.0
			m.pose.orientation.z=0.0
			m.pose.orientation.w=1.0
			m.lifetime=rospy.Duration()
			i=1
			while(i!=len(xarr)):
				po=Point()
			 	po.x=xarr[i]
			 	po.y=yarr[i]
			 	m.points.append(po)
			 	i+=1
			p.publish(m)
			#print("Marker", m.points)
		except:
			continue


def callback(msg):
	global ranges
	global c
	global s
	rangearr=msg.ranges
	rangearr=np.array(rangearr)
	rangearr[rangearr==3.0]=0
	x=rangearr*c
	y=rangearr*s
	ranges[:,0:1]=x.reshape((x.shape[0],1))
	ranges[:,1:]=y.reshape((y.shape[0],1))


if __name__ == '__main__':
	try:
		rospy.init_node('ransac',anonymous=True)
		ranges=np.zeros((361,2))
		rospy.Subscriber("base_scan",LaserScan,callback)
		deg=np.linspace(np.pi,-1 * np.pi,361)
		c=np.cos(deg)
		s=np.sin(deg)
		visaul()
	except rospy.ROSInterruptException as e:
		pass