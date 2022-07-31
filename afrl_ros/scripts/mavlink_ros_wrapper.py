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
from afrl_ros.msg import RollPitchYawIADS
# from nav_msgs import Odometry


def wrap_message(mavlink_msg):
    """wraps mavlink message"""
    mav_dict = msg.to_dict()
    rpy_msg = RollPitchYawIADS()
    rpy_msg.roll = math.degrees(mav_dict["roll"])
    rpy_msg.pitch = math.degrees(mav_dict["pitch"])
    rpy_msg.yaw = math.degrees(mav_dict["yaw"])
    
    iads_odom_pub.publish(rpy_msg)

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

    #connect to master might need to change this
    master = mavutil.mavlink_connection('udpin:0.0.0.0:14540')
    # # Make sure the connection is valid
    master.wait_heartbeat()

    while True:
        try:
            msg = master.recv_match()
            if not msg:
                continue
            if msg.get_type() == 'ATTITUDE':
                
                wrap_message(msg)
                # print("\n\n*****Got message: %s*****" % msg.get_type())
        
                # dict = msg.to_dict()
                # print("\n")
                # print("pitch is", dict["pitch"])

                # print("Message: %s" % msg)
                # print("\nAs dictionary: %s" % msg.to_dict())
                # # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
                # print("\nSystem status: %s" % msg.system_status)

            # rate.sleep()    

        except:
            pass


    # # Get some information !
    # while True:
    #     try:
    #         # print(master.recv_match().to_dict())
    #         msg = master.recv_match()
    #         if not msg:
    #             continue
    #         if msg.get_type() == 'ATTITUDE':

    #             print("\n\n*****Got message: %s*****" % msg.get_type())
        
    #             dict = msg.to_dict()
    #             print("\n")
    #             print("pitch is", dict["pitch"])

    #             print("Message: %s" % msg)
    #             print("\nAs dictionary: %s" % msg.to_dict())
    #             # Armed = MAV_STATE_STANDBY (4), Disarmed = MAV_STATE_ACTIVE (3)
    #             print("\nSystem status: %s" % msg.system_status)

    #     except:
    #         pass

