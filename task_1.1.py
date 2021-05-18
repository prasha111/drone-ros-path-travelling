#!/usr/bin/env python

#The required packages are imported here
from plutodrone.msg import *
from pid_tune.msg import *
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Int32
from std_msgs.msg import Float64
from multiprocessing import Process
import os
import rospy
import time



import rospy, cv2, cv_bridge
import numpy as np
from sensor_msgs.msg import	Image
from geometry_msgs.msg import Twist



class DroneFly():
	def __init__(self):
		
		rospy.init_node('pluto_fly', disable_signals = True)

		self.pluto_cmd = rospy.Publisher('/drone_command', PlutoMsg, queue_size=1000)

		rospy.Subscriber('whycon/poses', PoseArray, self.get_pose)

		# To tune the drone during runtime
		rospy.Subscriber('/pid_tuning_altitude', PidTune, self.set_pid_alt)
		rospy.Subscriber('/pid_tuning_roll', PidTune, self.set_pid_roll)
		rospy.Subscriber('/pid_tuning_pitch', PidTune, self.set_pid_pitch)
		rospy.Subscriber('/pid_tuning_yaw', PidTune, self.set_pid_yaw)
		
		self.cmd = PlutoMsg()

		
			# Position to hold.
		self.wp_x = -5.63
		self.wp_y = -5.63
		self.wp_z = 30.0
		self.wp1_x = 5.57
		self.wp1_y = -5.63
		self.wp1_z = 30
		self.wp2_x = 5.55
		self.wp2_y = 5.54
		self.wp2_z = 30
		self.wp3_x = -5.6
		self.wp3_y = 5.54
		self.wp3_z = 30
		self.wp4_x = 0.0
		self.wp4_y = 0.0
		self.wp4_z = 30
		self.i = 0

		self.f = 0.0
		self.ff = 0.0
		self.fff = 0.0

		
		self.cmd.rcRoll = 1500
		self.cmd.rcPitch = 1500
		self.cmd.rcYaw = 1500
		self.cmd.rcThrottle = 1500
		self.cmd.rcAUX1 = 1500
		self.cmd.rcAUX2 = 1500
		self.cmd.rcAUX3 = 1500
		self.cmd.rcAUX4 = 1000
		self.cmd.plutoIndex = 0

		self.drone_x = 0.0
		self.drone_y = 0.0
		self.drone_z = 0.0

		#PID constants for Roll
		self.kp_roll = 764.0
		self.ki_roll = 2.0
		self.kd_roll = 50.0

		#some extra define
		self.tp = 0
		self.p = 0 
		self.d = 0
		self.ie = 0 
		self.pp = 0.0
		self.pd = 0.0
		self.pie = 0.0
		self.rp = 0.0
		self.rd = 0.0
		self.rie = 0.0
		self.zero_line = 0.0
		self.yy = 0.0
		self.tp1 = 0.0
		self.tp3 = 0.0
		self.tp2 = 0.0
		self.yd = 0.0
		self.yp  = 0.0 
		self.yie = 0.0

		#PID constants for Pitch
		self.kp_pitch = 70.0
		self.ki_pitch = 0.0
		self.kd_pitch = 0.5
		
		#PID constants for Yaw
		self.kp_yaw = 50.0
		self.ki_yaw = 0.0
		self.kd_yaw = 0.0

		#PID constants for Throttle
		self.kp_throt = 55.0
		self.ki_throt = 0.0
		self.kd_throt = 220.0

		# Correction values after PID is computed
		self.correct_roll = 0.0
		self.correct_pitch = 0.0
		self.correct_yaw = 0.0
		self.correct_throt = 0.0

		# Loop time for PID computation. You are free to experiment with this
		self.last_time = 0.0
		self.loop_time = 0.1

		rospy.sleep(.1)


	def arm(self):
		self.cmd.rcAUX4 = 1500
		self.cmd.rcThrottle = 1000
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)

	def disarm(self):
		self.cmd.rcAUX4 = 1100
		self.pluto_cmd.publish(self.cmd)
		rospy.sleep(.1)


	def position_hold(self):

		rospy.sleep(2)

		print "disarm"
		self.disarm()
		rospy.sleep(.2)
		print "arm"
		self.arm()
		rospy.sleep(.1)


		while True:

			self.ip = 0

			self.setpoint = [self.wp_z, self.wp_x, self.wp_y, self.wp1_z, self.wp1_x, self.wp1_y, self.wp2_z, self.wp2_x, self.wp2_y, self.wp3_z, self.wp3_x, self.wp3_y, self.wp4_z, self.wp4_x, self.wp4_y ]

			self.error = [(self.drone_z - self.setpoint[self.i]), (self.drone_x - self.setpoint[(self.i) + 1]), (self.drone_y - self.setpoint[(self.i) + 2]), (0 -self.yy)]
			#print self.error[1]
			self.f = float(self.error[0])
			self.ff = float(self.error[1])
			self.fff = float(self.error[2])


			#self.setpoint = [self.wp_z, self.wp_x, self.wp_y ]

			#self.error = [(self.drone_z - self.setpoint[0]), (self.drone_x - self.setpoint[1]), (self.drone_y - self.setpoint[2]), (0 -self.yy)]
			#print self.error[0]
			self.calc_pid()
			rospy.Subscriber('/drone_yaw', Float64, self.data_yaw)

			# Check your X and Y axis. You MAY have to change the + and the -.
			# We recommend you try one degree of freedom (DOF) at a time. Eg: Roll first then pitch and so on
		 	pitch_value = int(1500 + self.correct_pitch)
			
			self.cmd.rcPitch = self.limit (pitch_value, 1600, 1400)
			



			roll_value = int(1500 + self.correct_roll)
			self.cmd.rcRoll = self.limit(roll_value, 1600,1400)
															
			throt_value = int(1500 + self.correct_throt)
			#print self.correct_throt
			self.cmd.rcThrottle = self.limit(throt_value, 1750,1350)

			yaw_value = int(1500 + self.correct_yaw)
			self.cmd.rcYaw = self.limit(yaw_value, 1510, 1490)
			#self.cmd.rcYaw = 1500-(self.yy)
			#print self.y

			self.pluto_cmd.publish(self.cmd.rcRoll, self.cmd.rcPitch, self.cmd.rcYaw, self.cmd.rcThrottle, self.cmd.rcAUX1, self.cmd.rcAUX2, self.cmd.rcAUX3, self.cmd.rcAUX4, self.cmd.plutoIndex)
			#print self.cmd
			pub0 = rospy.Publisher('/alt_error', Float64, queue_size = 1000)
			pub0.publish(self.error[0])

			pub1 = rospy.Publisher('/roll_error', Float64, queue_size = 1000)
			pub1.publish(self.error[2])

			pub2 = rospy.Publisher('/pitch_error', Float64, queue_size = 1000)
			pub2.publish(self.error[1])

			pub3 = rospy.Publisher('/zero_line', Float64, queue_size = 1000)
			pub3.publish(self.zero_line)

			pub4 = rospy.Publisher('/yaw_error', Float64, queue_size = 1000)
			pub4.publish(self.error[3])

			pub5 = rospy.Publisher('/YAW_THROTTLE', Float64, queue_size = 1000 )
			pub5.publish(self.correct_yaw)

			#pub9 = rospy.Publisher('whycon/image_out', Image)
			#pub9.publish('/whycon/image_out')


			if (self.f <= 1 and self.f >= -1 and self.ff >= -0.15 and self.ff <= 0.15 and self.fff >= -0.15 and self.fff <= 0.15 and self.i <=9):
				#for ip in range(0,1,1):
				self.i = self.i + 3
				#print self.i
				print 'a point travelled'

				#print "point travelled" , self.setpoint[i-3]
				#print self.setpoint[i-2]
				#print self.setpoint [i-1]
				continue
			elif (self.f <= 0.5 and self.f >= -0.5 and self.ff >= -0.2 and self.ff <= 0.2 and self.fff >= -0.2 and self.fff <= 0.2 and self.i >= 12):

				self.disarm()
				print 'drone travelled last  distance '
				#print "point travelled" , setpoint[i-3]
				#print setpoint[i-2]
				#print setpoint [i-1]

			else:
				continue
				



			

	def calc_pid(self):
		self.seconds = time.time()
		self.current_time = self.seconds - self.last_time
		if(self.current_time >= self.loop_time):
			self.pid_roll()
			self.pid_pitch()
			self.pid_throt()
			self.pid_yaw()
			
			self.last_time = self.seconds


	def pid_roll(self):

		while(self.error[2]!=0):

			self.rp = (self.error[2])*(self.kp_roll/100)
			self.rd = ((self.error[2]-self.tp2)/self.current_time)*(self.kd_roll/100)
			self.rie = (self.error[2]*self.current_time)*(self.ki_roll)
			self.correct_roll = self.rp + self.rd + self.rie
			self.tp2 = self.error[2]
			#print self.correct_roll
			return self.correct_roll



	def pid_pitch(self):

		while(self.error[1]!=0):
			self.pp = (self.error[1]*self.kp_pitch)/10
			self.pd = ((self.error[1]-self.tp1)/self.current_time)*(self.kd_pitch/100)
			self.pie = (self.error[1]*self.current_time)*self.ki_pitch/10
			self.correct_pitch = self.pp + self.pd + self.pie
			
			self.tp1 = self.error[1]
			#print self.error[1]
			return self.correct_pitch




	def pid_throt(self):
		while(self.error[0] != 0):
			
			self.p = self.error[0]*self.kp_throt/10
			self.d = ((self.error[0]-self.tp)/self.current_time)*(self.kd_throt/10)
			self.ie = (self.error[0]*self.current_time)*(self.ki_throt)
			self.correct_throt = self.p + self.d + self.ie
			
			self.tp  = self.error[0]
			#print self.error[0]
			return self.correct_throt

			
		

	def pid_yaw(self):

		while(self.error[3] != 0):
			self.yp = self.error[3]*(self.kp_yaw/10)
			self.yd = ((self.error[3]-self.tp3)/self.current_time)*(self.kd_yaw/100)
			self.yie = ((self.error[3]*self.current_time)*(self.ki_yaw/100))
			self.correct_yaw = self.yp + self.yd + self.yie
			self.tp3 = self.error[3]
			#print self.error[1]
			
			return self.correct_yaw




	def limit(self, input_value, max_value, min_value):

		#Use this function to limit the maximum and minimum values you send to your drone

		if input_value > max_value:
			return max_value
		if input_value < min_value:
			return min_value
		else:
			return input_value

	#You can use this function to publish different information for your plots
	#def publish_plot_data(self):
	def data_yaw(self,yaw_value1):
		self.yy = yaw_value1.data
		#print self.yy

	def set_pid_alt(self,pid_val):
		
		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Altitude

		self.kp_throt = pid_val.Kp
		self.ki_throt = pid_val.Ki
		self.kd_throt = pid_val.Kd

	def set_pid_roll(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Roll

		self.kp_roll = pid_val.Kp
		self.ki_roll = pid_val.Ki
		self.kd_roll = pid_val.Kd
		
	def set_pid_pitch(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Pitch

		self.kp_pitch = (pid_val.Kp)
		self.ki_pitch = pid_val.Ki
		self.kd_pitch = pid_val.Kd
		
	def set_pid_yaw(self,pid_val):

		#This is the subscriber function to get the Kp, Ki and Kd values set through the GUI for Yaw

		self.kp_yaw = pid_val.Kp
		self.ki_yaw = pid_val.Ki
		self.kd_yaw = pid_val.Kd
		
	def get_pose(self,pose):

		#This is the subsccriber function to get the whycon poses
		#The x, y and z values are stored within the drone_x, drone_y and the drone_z variables
		
		self.drone_x = pose.poses[0].position.x

		self.drone_y = pose.poses[0].position.y

		self.drone_z = pose.poses[0].position.z
		#print self.drone_z


if __name__ == '__main__':
	while not rospy.is_shutdown():
		temp = DroneFly()
		temp.position_hold()
		rospy.spin()
