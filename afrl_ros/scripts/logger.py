#!/usr/bin/env python
# -*- coding: utf-8 -*- 

import rospy

import time
import csv
import os
import numpy as np
import datetime
from std_msgs.msg import Int8
from mavros_msgs.msg import FlightTestInput
from geometry_msgs.msg import PoseStamped,TwistStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion



class Attitude():
	def __init__(self, topic_name) -> None:
		#super().__init__(limit_range)
		
		self.rpy_radians = [None, None, None]  
		self.rpy_rate = [None, None, None]
		self.rpy_deg = [None, None, None]
		self.rpy_rate_deg = [None, None, None]
		self.odom_sub = rospy.Subscriber(topic_name, Odometry, self.odom_cb)


	def odom_cb(self, msg):
		"""callback for odometry data"""
		orientation_q = msg.pose.pose.orientation
		rpy_rate = msg.twist.twist.angular
		self.rpy_rate = [rpy_rate.x, rpy_rate.y, rpy_rate.z] 
		self.rpy_rate_deg = np.rad2deg(self.rpy_rate)

		#orientation
		orientation_list = [orientation_q.x, orientation_q.y,
							 orientation_q.z, orientation_q.w]
		self.rpy_radians = euler_from_quaternion(orientation_list)
		self.rpy_deg = np.rad2deg(self.rpy_radians)
		
	
class FlightTest():
	def __init__(self, sub_topic):
		self.sub = rospy.Subscriber(sub_topic, FlightTestInput, self.fti_cb)
		self.fti_mode = None
		self.fti_state = None
		self.fti_sweep_time_segment_pct = None
		self.fti_sweep_frequency = None
		self.fti_sweep_amplitude = None
		self.fti_injection_input = None
		self.fti_injection_output = None
		self.fti_raw_output = None
		self.fti_injection_point = None

	def fti_cb(self, msg):
		"""get flight test data back"""
		self.fti_mode = msg.fti_mode
		self.fti_state = msg.fti_state
		self.fti_sweep_time_segment_pct = msg.fti_sweep_time_segment_pct
		self.fti_sweep_frequency = msg.fti_sweep_frequency
		self.fti_sweep_amplitude = msg.fti_sweep_amplitude
		self.fti_injection_input = msg.fti_injection_input
		self.fti_injection_output = msg.fti_injection_output
		self.fti_raw_output = msg.fti_raw_output
		self.fti_injection_point = msg.fti_injection_point


if __name__ == '__main__':
	
	quad_topic = "mavros/odometry/in"
	quad = Attitude(quad_topic)

	flight_test_topic = "mavros/flight_test_input"
	flight_test = FlightTest(flight_test_topic)

	rospy.init_node('logger', anonymous=True)

	myData = ["time", 
	"roll", 
	"pitch", 
	"yaw", 
	"roll_rate",
	"pitch_rate", 
	"yaw_rate", 
	"fti_mode", 
	"fti_state", 
	"fti_sweep_percent", 
	"fti_sweep_freq", 
	"fti_sweep_amp", 
	"fti_injection_input", 
	"fti_injection_output", 
	"fti_raw_output",
	"fti_injection_point"] 

	fileNameBase = "/home/justin/catkin_ws/src/afrl_ros/afrl_ros/scripts/logfiles/" + "flight_test" + "_"+ datetime.datetime.now().strftime("%b_%d_%H_%M")
	fileNameSuffix = ".csv"
	# num is used for incrementing the file path if we already have a file in the directory with the same name
	num = 1
	fileName = fileNameBase + fileNameSuffix
	# check if the file already exists and increment num until the name is unique
	while os.path.isfile(fileName):
		fileName = fileNameBase + "_" + str(num)+"_" + "kf_q" + fileNameSuffix
		num = num + 1


	# now we know we have a unique name, let's open the file, 'a' is append mode, in the unlikely event that we open
	# a file that already exists, this will simply add on to the end of it (rather than destroy or overwrite data)
	myFile = open(fileName, 'a')
	with myFile:
		writer = csv.writer(myFile)
		writer.writerow(myData)

	# get the CPU time at which we started the node, we will use this to subtract off so that our time vector
	# starts near 0
	zero_time = rospy.get_time()

	# this is some ros magic to control the loop timing, you can change this to log data faster/slower as needed
	# note that the IMU publisher publishes data at a specified rate (500Hz) and while this number could be
	# changes, in general, you should keep the loop rate for the logger below the loop rate for the IMU publisher
	rate = rospy.Rate(20) #100 Hz
	# try/except block here is a fancy way to allow code to cleanly exit on a keyboard break (ctrl+c)
	try:
		while not rospy.is_shutdown():
			# get the current time and subtract off the zero_time offset
			now = (rospy.get_time()-zero_time)
			# create the data vector which we will write to the file, remember if you change
			# something here, but don't change the header string, your column headers won't
			# match the data
			myData = [now, 
			quad.rpy_deg[0],
			quad.rpy_deg[1],
			quad.rpy_deg[2], 
			quad.rpy_rate_deg[0], 
			quad.rpy_rate_deg[1],
			quad.rpy_rate_deg[2],
			flight_test.fti_mode, 
			flight_test.fti_state, 
			flight_test.fti_sweep_time_segment_pct,
			flight_test.fti_sweep_frequency, 
			flight_test.fti_sweep_amplitude, 
			flight_test.fti_injection_input , 
			flight_test.fti_injection_output, 
			flight_test.fti_raw_output,
			flight_test.fti_injection_point]
   
			# for kf in kf_est_list:
			# 	myData.append(kf[0].coords)
			# 	myData.append(kf[1].coords)
	
			# stick everything in the file
			myFile = open(fileName, 'a')
			with myFile:
				writer = csv.writer(myFile)
				writer.writerow(myData)

			# this is ros magic, basically just a sleep function with the specified dt
			rate.sleep()

	# as stated before, try/except is used to nicely quit the program using ctrl+c
	except rospy.ROSInterruptException:
		pass
