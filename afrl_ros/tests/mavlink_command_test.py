"""
Example of how to connect pymavlink to an autopilot via an UDP connection
"""

# Disable "Bare exception" warning
# pylint: disable=W0702

import time
# Import mavutil
import sys
from pymavlink import mavutil
# Import ardupilotmega module for MAVLink 1
from pymavlink.dialects.v10 import common as mavlink1

# Import common module for MAVLink 2
# from pymavlink.dialects.v20 import common as mavlink2

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

master = mavutil.mavlink_connection('udpin:0.0.0.0:14540')

# Make sure the connection is valid
master.wait_heartbeat()

# # Choose a mode
# mode = 'MISSION'

# # Check if mode is available
# if mode not in master.mode_mapping():
#     print('Unknown mode : {}'.format(mode))
#     print('Try:', list(master.mode_mapping().keys()))
#     sys.exit(1)

# # Get mode ID
# mode_id = master.mode_mapping()[mode]
# # Set new mode
# master.set_mode(mode_id) 

# Get mode ID
# mode = 'TAKEOFF'
# mode_id = master.mode_mapping()[mode]
# print("mode id", mode_id)
# Set new mode


"""set  airspeed"""
print(master.target_system)
print(master.target_component)
airspeed = 19 #m/s
des_alt = 55
base_alt = 702.0

master.mav.command_long_send(
master.target_system, 
master.target_component,
mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
0, #confirmation
0, #Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
airspeed, #Speed #m/s
-1, #Throttle (-1 indicates no change) % 
0, 0, 0, 0 #ignore other parameters
)
print("change airspeed")


"""
Which waypoints are which?
Level 1 75m -- 0
Level 2 100m -- 5
Level 3 125m -- 10
Level 4 150m -- 15
"""

wp_num = 15
master.waypoint_set_current_send(wp_num)


while True:
    try:
        time.sleep(0.2)
        master.waypoint_set_current_send(wp_num)
        wps = master.waypoint_current()
        print("wps", wps)

        master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
        0, #confirmation
        0, #Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
        airspeed, #Speed #m/s
        -1, #Throttle (-1 indicates no change) % 
        0, 0, 0, 0 #ignore other parameters
        )

        if wps == wp_num:
            print("breaking")
            break

    except KeyboardInterrupt:
        master.mav.command_long_send(
        master.target_system, 
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, #command
        0, #confirmation
        0, #Speed type (0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed)
        15, #Speed #m/s
        -1, #Throttle (-1 indicates no change) % 
        0, 0, 0, 0 #ignore other parameters
        )
        print("slowing down")

        # master.mav.command_long_send(
        #     master.target_system,
        #     master.target_component,
        #     mavutil.mavlink.MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT, #command
        #     0,
        #     0, #0=within 5m of desired alt, 1 at or above alt, 2 at or below alt
        #     0, 
        #     0,
        #     0,
        #     0,
        #     0,
        #     45 #desired alt m
        # )
        # print("descending")

        break


