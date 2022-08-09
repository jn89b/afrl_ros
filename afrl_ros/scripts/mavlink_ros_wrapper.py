#!/usr/bin/env python
"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
import rospy
import math 
# Import mavutil
from pymavlink import mavutil
from afrl_ros.msg import RollPitchYawIADS, HeadingAirspeedAltitudeIADS
from afrl_configs import mav_ports

def wrap_iads_rpy(mavlink_msg):
    """wraps mavlink message and publishes to ROS Topic"""
    mav_dict = mavlink_msg.to_dict()
    rpy_msg = RollPitchYawIADS()
    rpy_msg.roll = math.degrees(mav_dict["roll"])
    rpy_msg.pitch = math.degrees(mav_dict["pitch"])
    rpy_msg.yaw = math.degrees(mav_dict["yaw"])

    rpy_msg.rollspeed = math.degrees(mav_dict["rollspeed"])
    rpy_msg.pitchspeed = math.degrees(mav_dict["pitchspeed"])
    rpy_msg.yawspeed = math.degrees(mav_dict["yawspeed"])
    
    iads_odom_pub.publish(rpy_msg)

def wrap_iads_vfr(mavlink_msg):
    mav_dict = mavlink_msg.to_dict()
    vfr_msg = HeadingAirspeedAltitudeIADS()
    vfr_msg.heading = mav_dict["heading"]
    vfr_msg.airspeed = mav_dict["airspeed"]
    vfr_msg.altitude = mav_dict["alt"]

    iads_vfr_pub.publish(vfr_msg)


if __name__ == '__main__':
        
    # Create the connection
    #  If using a companion computer
    #  the default connection is available
    #  at ip 192.168.2.1 and the port 14540
    # Note: The connection is done with 'udpin' and not 'udpout'.
    #  You can check in http:192.168.2.2:2770/mavproxy that the communication made for 14550
    #  uses a 'udpbcast' (client) and not 'udpin' (server).
    #  If you want to use QGroundControl in parallel with your python script,
    #  it's possible to add a new output port in http:192.168.2.2:2770/mavproxy as a new line.
    #  E.g: --out udpbcast:192.168.2.255:yourport

    rospy.init_node("mavlink_ros_wrapper")
    rate_val = 30
    rate = rospy.Rate(rate_val)

    iads_odom_pub = rospy.Publisher("iads_odom", RollPitchYawIADS, queue_size=1)
    iads_vfr_pub = rospy.Publisher("iads_vfr", HeadingAirspeedAltitudeIADS, queue_size=1)

    #connect to master might need to change this
    master = mavutil.mavlink_connection('udpin:0.0.0.0:'+mav_ports.MAV_WRAPPER_PORT)
    # # Make sure the connection is valid
    master.wait_heartbeat()

    while True:
        try:

            msg = master.recv_match()
            if not msg:
                continue

            if msg.get_type() == 'ATTITUDE': 
                wrap_iads_rpy(msg)

            if msg.get_type() == 'VFR_HUD':
                #mav_dict = msg.to_dict()
                # print(msg)
                wrap_iads_vfr(msg)
                
        except:
            pass
