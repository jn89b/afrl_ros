#!/usr/bin/env python

import socket
from xmlrpc.client import boolean
# from threading import Thread
from supervisor import PTI
from supervisor import WaypointObserver
import threading


import rospy
import mavros
import math

from nav_msgs.msg import Odometry
from mavros_msgs.msg import  HomePosition, WaypointList
from sensor_msgs.msg import NavSatFix

from tf.transformations import euler_from_quaternion, quaternion_from_euler

class FlightCardListener():
    def __init__(self):
        self.port = PORT 
        self.max_connections = MAXCONNECTIONS
        self.correct_msg_len = 6
        #self.client_thread = Thread(target=self.listen_client, args=())


    def is_msg_correct(self, client_msg)-> boolean:
        """check udp message parameters"""    
        if len(client_msg) == self.correct_msg_len:
            return True
        else:
            return False

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

if __name__=='__main__':

    # rospy.init_node("flight_card_listener", anonymous=False)
    # rate_val = 10
    # rate = rospy.Rate(rate_val)
    mavros.set_namespace()

    PORT = 9876
    MAXCONNECTIONS = 2
    THIS_IP = "10.3.7.205"
    # THIS_IP = "192.168.231.110"
    BUFFSIZE = 1024

    listensocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listensocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
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

            if fc_listener.is_msg_correct(message):

                #check if we SHOULD do this test based on prechecks
                good_message = message
                pti_dict = fc_listener.parse_msg(good_message)

                #if waypoint_observe.pre_no_go(pti_dict["FTI_FS_DURATION"]) == True:

                pti_verify.set_injection_vals("FTI_INJXN_POINT", 
                            pti_dict["FTI_INJXN_POINT"], pti_dict["FTI_FS_AMP_BEGIN"])

                pti_verify.set_pti_param("FTI_MODE", pti_dict["FTI_MODE"])
                pti_verify.set_pti_param("FTI_FS_DURATION", pti_dict["FTI_FS_DURATION"])
                print(pti_dict["FTI_LOOP_GAIN"])

                pti_verify.set_loop_gain_param(pti_dict["FTI_LOOP_GAIN"])
                pti_verify.set_pti_param("FTI_ENABLE", pti_dict["FTI_ENABLE"])
                
                
                
                # else:
                #     print("no go")

            elif "END" in message:
                print("test is ending", message)
                pti_verify.set_pti_param("FTI_ENABLE", 0)
            
            else:
                print("bad inputs")

            print("\n---------------------")

        except IOError:
            print("io error")
            continue
        # rate.sleep()
            





        
