#!/usr/bin/env python
import rospy
from mavros_msgs.srv import SetMode
from mavros_msgs.srv import CommandBool
from mavros_msgs.srv import CommandTOL
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import OverrideRCIn
from mavros_msgs.msg import ParamValue
from mavros_msgs.srv import ParamSet

class Operation():

	def state_callback(self,data):
		self.cur_state = data

	def gps_callback(self,data):
		self.gps = data
		self.gps_read = True

	def start_operation(self):
		rospy.init_node('my_operation', anonymous=True)	
		self.gps_read = False
		self.localtarget_received = False
		r = rospy.Rate(10)
		rospy.Subscriber("/mavros/state", State, self.state_callback)
		rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
		pub_rc = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)

		while not self.gps_read:
			r.sleep()
		
		# Service Clients
		change_mode = rospy.ServiceProxy('/mavros/set_mode', SetMode) 
		arm = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
		takeoff = rospy.ServiceProxy('/mavros/cmd/takeoff', CommandTOL)
		change_param = rospy.ServiceProxy('/mavros/param/set', ParamSet)
		
		last_request = rospy.get_rostime() 

		# Change mode to GUIDED
		rospy.wait_for_service('/mavros/set_mode')
		try:
			base_mode = 0
			custom_mode = "GUIDED"
			out = change_mode(base_mode, custom_mode)
			if out.success:
				rospy.loginfo("GUIDED mode set")
			else:
				rospy.loginfo("Failed SetMode")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		last_request = rospy.get_rostime() 
		
		while not out.success:
			r.sleep()
			out = change_mode(base_mode, custom_mode)
			if out.success:
				rospy.loginfo("setmode send ok value")
			else:
				rospy.loginfo("Failed SetMode")

		# Arm drone
		rospy.wait_for_service('/mavros/cmd/arming')
		try:
			out = arm(True)
			if out.success:
				rospy.loginfo("Armed")
			else:
				rospy.loginfo("Failed Arming")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		last_request = rospy.get_rostime() 
		
		while not out.success:
			r.sleep()
			out = arm(True)
			if out.success:
				rospy.loginfo("Armed")
			else:
				rospy.loginfo("Failed Arming")

		# Take off
		current_altitude = self.gps.altitude 
		rospy.wait_for_service('/mavros/cmd/takeoff')
		try:			
			min_pitch = 0
			yaw = 0
			latitude = 0 #self.gps.latitude
			longitude = 0 #self.gps.longitude
			altitude = 4
			out = takeoff(min_pitch, yaw, latitude, longitude, altitude)
			if out.success:
				rospy.loginfo("Took-off")
			else:
				rospy.loginfo("Failed taking-off")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		
		# Keep sending take-off messages until received by FCU
		while not out.success:
			r.sleep()
			out = takeoff(min_pitch, yaw, latitude, longitude, altitude)
			if out.success:
				rospy.loginfo("Took-off")
			else:
				rospy.loginfo("Failed taking-off")
		
		while self.gps.altitude< current_altitude+altitude-0.1:
			r.sleep()
			differ = self.gps.altitude - current_altitude
			rospy.loginfo("Waiting to take off, current height %s", differ)

		# Change GCS parameter
		rospy.wait_for_service('/mavros/param/set')
		try:			
			myparam = ParamValue()
			myparam.integer = 1
			out = change_param("SYSID_MYGCS", myparam)
			if out.success:
				rospy.loginfo("Changed my GCS")
			else:
				rospy.loginfo("Failed changing GCS")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")

		# Set throttle to 1500 (not controlled in circle mode)
		myrc = OverrideRCIn()
		myrc.channels[2] = 1500
		pub_rc.publish(myrc)
		rospy.sleep(3.)

		# Change GCS parameter back
		try:
			rospy.wait_for_service('/mavros/param/set')
			myparam = ParamValue()
			myparam.integer = 255
			myparam.real = 0
			out = change_param("SYSID_MYGCS", myparam)
			if out.success:
				rospy.loginfo("Changed my GCS back")
			else:
				rospy.loginfo("Failed changing GCS")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")

		# Change RTL parameters
		rospy.wait_for_service('/mavros/param/set')
		try:
			myparam.integer = 0
			myparam.real = 4.0
			out = change_param("RTL_ALT", myparam)
			if out.success:
				rospy.loginfo("Changed rtl altitude")
			else:
				rospy.loginfo("Failed changing rtl altitude")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		
		# Change circle parameters
		rospy.wait_for_service('/mavros/param/set')
		try:
			myparam.integer = 0
			myparam.real = 1.0 # now in deg/s!
			out = change_param("CIRCLE_RATE", myparam)
			if out.success:
				rospy.loginfo("Changed circle rate")
			else:
				rospy.loginfo("Failed changing circle rate")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		
		try:
			myparam.integer = 0
			myparam.real = 1000.0 # now in cm!
			out = change_param("CIRCLE_RADIUS", myparam)
			if out.success:
				rospy.loginfo("Changed circle radius")
			else:
				rospy.loginfo("Failed changing circle radius")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")
		
		# Change mode to circle
		rospy.wait_for_service('/mavros/set_mode')
		try:
			rospy.sleep(10.)
			base_mode = 0
			custom_mode = "CIRCLE"
			out = change_mode(base_mode, custom_mode)
			if out.success:
				rospy.loginfo("CIRCLE mode set")
			else:
				rospy.loginfo("Failed setting CIRCLE mode")
		except rospy.ServiceException, e:
			rospy.loginfo("Service call failed")

    if __name__ == '__main__':
    	my_operation = Operation()
    	my_operation.start_operation()