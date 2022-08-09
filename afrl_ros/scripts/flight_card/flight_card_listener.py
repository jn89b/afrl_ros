#!/usr/bin/env python

from multiprocessing import current_process
import socket
from xmlrpc.client import boolean
# from threading import Thread
from afrl_configs import performance_config, mav_ports
from supervisor import PTI
from supervisor import WaypointObserver
import threading


import rospy
import mavros
import math

import time
import sys
from pymavlink import mavutil
from pymavlink.dialects.v10 import common as mavlink1

from nav_msgs.msg import Odometry
from mavros_msgs.msg import  HomePosition, WaypointList
from sensor_msgs.msg import NavSatFix

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class FlightCardListener():
    def __init__(self):
        self.port = PORT 
        self.max_connections = MAXCONNECTIONS
        self.correct_msg_len = 6
        self.correct_perf_len = 5
        #self.client_thread = Thread(target=self.listen_client, args=())


    def is_msg_correct(self, client_msg)-> boolean:
        """check udp message parameters"""    
        if len(client_msg) == self.correct_msg_len:
            return True
        else:
            return False

    def is_perform_msg_correct(self, client_msg) -> boolean:
        if len(client_msg) == self.correct_perf_len:
            return True
        else:
            return False

    def parse_perform_msg(self, client_msg) -> list:
        time_duration = float(client_msg[1])
        velocity = float(client_msg[2])
        airspeed_key = client_msg[3]
        altitude_key = client_msg[4]
        #loop_gain_setting = client_msg[5]

        return [airspeed_key, altitude_key]

    def parse_msg(self, client_msg)-> dict:
        """parse message and return as dictionary"""
        if client_msg[0] == "FREQ_SWEEP":
            fti_mode_val = 0

        time_duration = float(client_msg[1])
        velocity = float(client_msg[2])
        injection_point = int(client_msg[3])
        amp_setting = client_msg[4]
        loop_gain_setting = client_msg[5]

        #need to update android side
        pti_dict = {"FTI_MODE": fti_mode_val,
                    "FTI_FS_DURATION": time_duration,
                    "FTI_INJXN_POINT": injection_point,
                    "FTI_FS_AMP_BEGIN": amp_setting,
                    "FTI_FS_AMP_END": amp_setting,
                    "FTI_LOOP_GAIN": loop_gain_setting,
                    "FTI_ENABLE": int(1)}

        return pti_dict

def close_socket(socket:socket):
    print("closing socket", socket)
    socket.close()

def send_airspeed_command(airspeed,master):
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


if __name__=='__main__':

    # rospy.init_node("flight_card_listener", anonymous=False)
    # rate_val = 10
    # rate = rospy.Rate(rate_val)
    mavros.set_namespace()
    
    # port_number = '14520'
    master = mavutil.mavlink_connection('udpin:0.0.0.0:'+mav_ports.FLIGHT_CARD_PORT)
    # Make sure the connection is valid
    master.wait_heartbeat()
    print("got heartbeat")

    PORT = 9876
    MAXCONNECTIONS = 2
    THIS_IP = "10.3.7.205"
    # THIS_IP = "192.168.231.110"
    BUFFSIZE = 1024

    listensocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listensocket.setsockopt(socket.SOL_SOCKET, socket.mav_wrapper_portSO_REUSEADDR, 1)
    listensocket.bind((THIS_IP, PORT))

    #open server
    listensocket.listen(MAXCONNECTIONS)

    OLD_ENABLE_VAL = None

    fc_listener = FlightCardListener()
    pti_verify = PTI.PTIParamVerifier()
    waypoint_observe = WaypointObserver.WaypointObserver()

    # time.sleep(5)

    #while not rospy.is_shutdown(): 
    while True:
        # clientsocket = None 

        try:      
            clientsocket,clientaddress = listensocket.accept()
            message_packet = clientsocket.recv(BUFFSIZE).decode()
            clientsocket.setblocking(0)
            message = message_packet.split()
            print("message is", message)

            if "FREQ_SWEEP" in message:

                if fc_listener.is_msg_correct(message):

                    #check if we SHOULD do this test based on prechecks
                    good_message = message
                    pti_dict = fc_listener.parse_msg(good_message)

                    #if waypoint_observe.pre_no_go(pti_dict["FTI_FS_DURATION"]) == True:

                    pti_verify.set_injection_vals("FTI_INJXN_POINT", 
                                pti_dict["FTI_INJXN_POINT"], pti_dict["FTI_FS_AMP_BEGIN"])

                    pti_verify.set_pti_param("FTI_MODE", pti_dict["FTI_MODE"])
                    pti_verify.set_pti_param("FTI_FS_DURATION", pti_dict["FTI_FS_DURATION"])
                    # print(pti_dict["FTI_ENABLE"])
                    
                    pti_verify.set_loop_gain_param(pti_dict["FTI_LOOP_GAIN"])
                    pti_verify.set_pti_param("FTI_ENABLE", pti_dict["FTI_ENABLE"])

                if "END" in message:
                    print("test is ending", message)
                    pti_verify.set_pti_param("FTI_ENABLE", 0)

            elif "PERFORMANCE" in message:
                print("its performance")

                if fc_listener.is_perform_msg_correct(message):
                    airspeed_key, alt_key = fc_listener.parse_perform_msg(message)
                    print("retrieved messages: ", airspeed_key, alt_key)
                    
                    if alt_key not in performance_config.AGL_CONFIG:
                        print("no alt key")
                        continue

                    if airspeed_key not in performance_config.AIRSPEED_CONFIG:
                        print("no airspeed")
                        continue

                    print("alt key ", performance_config.AGL_CONFIG[alt_key])
                    print("airspeed key ", performance_config.AIRSPEED_CONFIG[airspeed_key])

                    airspeed_val = performance_config.AIRSPEED_CONFIG[airspeed_key]
                    des_wp = performance_config.AGL_CONFIG[alt_key]
                    curr_wp = master.waypoint_current()

                    send_airspeed_command(airspeed_val, master)

                    ## check if in same level of desired waypoint
                    curr_wp = master.waypoint_current()
                    des_level = performance_config.AGL_LEVEL[alt_key]

                    if curr_wp in des_level:
                        print("at desired level", curr_wp, des_level)
                        print("\n---------------------")
                        continue
                    
                    else:
                        while True:
                            time.sleep(0.2)
                            master.waypoint_set_current_send(des_wp)
                            curr_wp = master.waypoint_current()

                            if curr_wp == des_wp:
                                print("going to waypoint")
                                break
            
            else:
                print("bad inputs")

            print("\n---------------------")

        except IOError:
            print("io error")
            continue
        # rate.sleep()
            





        
