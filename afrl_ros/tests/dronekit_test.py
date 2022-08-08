from dronekit import connect, Command, LocationGlobal
from pymavlink import mavutil
import time, sys, argparse, math
import mavsdk

def PX4setMode(mode_list):
    vehicle._master.mav.command_long_send(vehicle._master.target_system, 
                                        vehicle._master.target_component,
                                        mavutil.mavlink.MAV_CMD_DO_SET_MODE, 
                                        0,
                                        mode_list[0],
                                        mode_list[1], 
                                        mode_list[2], 
                                        0, 
                                        0, 
                                        0, 
                                        0)
                                               



# Connect to the Vehicle
# print("connecting")
# connection_string = '127.0.0.1:14540'
# vehicle = connect(connection_string, wait_ready=True)

# hold_mode = [217, 4, 3]
# mission_mode = [157, 4, 4]

# BASE_MODE = mission_mode[0]
# MAIN_MODE = mission_mode[1]
# SUB_MODE = mission_mode[2]

# # Change to AUTO mode
# PX4setMode(hold_mode)
# time.sleep(1)

# # Load commands
# cmds = vehicle.commands
# cmds.clear()


