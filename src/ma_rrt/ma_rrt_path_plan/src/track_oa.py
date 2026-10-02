#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import time as tm
import rospy
from math import*
from control_msgs.msg import Velocity
from morai_msgs.msg import CtrlCmd
from std_msgs.msg import Float32
from nav_msgs.msg import Path,Odometry

class makePath() :

	def __init__(self) :
		path_pub = rospy.Publisher('/track_path', Path, queue_size = 1)
		rospy.Subscriber("/ERP42_velocity", Velocity, self.velocity_callback)
		rospy.Subscriber("/ERP42_steer_conv", Float32, self.steer_callback)

		position_x = 0						#current_position initialization
		position_y = 0						
		v = Velocity()
		self.steer_conv_msg = 0
		steer = self.steer_conv_msg
		self.curvel_conv_msg = 0
		delta = steer*(pi/180)				#delta subscribe check
		pre_time = 0

		while not rospy.is_shutdown():
			time = tm.time()
			dt = time - pre_time
			v = self.curvel_conv_msg
			d = v*dt
			print("delta : ", delta)

			if (-28 <= delta and delta < 0):
				delta = abs(delta)
				position_x += d*cos(delta)
				position_y += d*sin(delta)

			elif (delta == 0):
				delta = 0
				position_x += d
				position_y = position_y

			elif (0 < delta and delta <= 28):
				delta = delta
				position_x += d*cos(delta)
				position_y -= d*sin(delta)

			print ("point : ", (position_x, position_y))
			#print("time : ", time.time()-start)	#time difference

			pre_time = time
			path_pub.publish()

		
	def velocity_callback(self, velocity) :
		self.curvel_msg=Velocity()					#Velocity message
		self.curvel_msg=velocity 

	def steer_callback(self, data) :
		self.steer_conv_msg=data		



if __name__ == '__main__':
	try: 
		rospy.init_node('ERP42_track_mission')
		makepath = makePath()

	except rospy.ROSInterruptException :
		pass
