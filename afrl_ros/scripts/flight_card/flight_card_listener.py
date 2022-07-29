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



# class WaypointObserver(threading.Thread):
#     def __init__(self):
#         threading.Thread.__init__(self)
#         rospy.init_node("flight_card_listener", anonymous=False)

#         self.home_position = HomePosition()
#         self.global_position = NavSatFix()
#         self.mission_wp = WaypointList()
        
#         #SUBSCRIBERS
#         self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
#                                              HomePosition,
#                                              self.home_position_callback)

#         self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
#                                                NavSatFix,
#                                                self.global_position_callback)

#         self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', 
#                                                 WaypointList, 
#                                                 self.mission_wp_callback)

#         self.airspeed_sub = rospy.Subscriber('mavros/odometry/in',
#                                             Odometry, 
#                                             self.airspeed_cb)
#         """"""
#         # self.prev_pos = [None, None]
#         # self.curr_pos = [None, None]
#         # self.airspeed = None

#         ## the overall no gos
#         # self.no_go = True

#     def home_position_callback(self, data):
#         self.home_position = data

#     def global_position_callback(self, data):
#         """get gps data"""
#         self.global_position = data
#         self.curr_pos = data.latitude, data.longitude
    

#     def mission_wp_callback(self, data):
#         if self.mission_wp.current_seq != data.current_seq:
#             rospy.loginfo("current mission waypoint sequence updated: {0}".
#                           format(data.current_seq))

#         self.mission_wp = data

#     def airspeed_cb(self,data): 
#         """get body airspeed of system"""
#         self.airspeed = data.twist.twist.linear.x 


#     def distance_to_wp(self, lat:float, lon:float, alt:float):
#         """alt(amsl): meters"""
#         R = 6371000  # metres
#         rlat1 = math.radians(lat)
#         rlat2 = math.radians(self.global_position.latitude)

#         rlat_d = math.radians(self.global_position.latitude - lat)
#         rlon_d = math.radians(self.global_position.longitude - lon)

#         #haversine equation
#         a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
#              math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
#         c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

#         d = R * c
#         # alt_d = abs(alt - self.altitude.amsl)

#         #rospy.logdebug("d: {0}, alt_d: {1}".format(d))#, alt_d))
#         return d

#     def check_bad_distance(self, pti_duration_time:float) -> boolean:
#         """check if possible to do the pti during the duration, by 
#         multiplying the current set velocity by the time to get the distance, 
#         see how much distance is left to the next waypoint, if greater than we cant do it"""
        
#         lat_dist_desired = self.airspeed * pti_duration_time
#         current_wp = self.mission_wp.waypoints[self.mission_wp.current_seq]
#         remain_lat_d = self.distance_to_wp(current_wp.x_lat, 
#                                             current_wp.y_long,
#                                             current_wp.z_alt) 
#         # print("lat desired is ", lat_dist_desired, "remaining", remain_lat_d)
#         if abs(lat_dist_desired) >= abs(remain_lat_d):
#             rospy.loginfo("too close to current distance dist_remaining: {0:.9f},{0:.9f},{0:.9f}".
#             format(remain_lat_d), (lat_dist_desired), (remain_lat_d))
#             return True
#         else:
#             return False

#     def compute_vector(self, point_1, point_2):
#         """given two points compute the vector"""
#         some_vec = np.array(point_2)- np.array(point_1)
#         return some_vec

#     def gps_to_xy(self, lat:float, lon:float):
#         """converts gps coordinates to x y coordinates in meters"""
#         R = 6371000  # metres
#         rlat = math.radians(lat)
#         rlon = math.radians(lon)
#         # rlat2 = math.radians(self.global_position.latitude)
#         x = R * math.cos(rlat) * math.cos(rlon)
#         y = R * math.cos(rlat) * math.sin(rlon)

#         return [x,y]

#     def check_curve_path(self):
#         """
#         check if vector directions are not straight returns True if so
#         if I take the dot product between two vectors and get 1 then its straight
#         if I take the cross product between two vectors and get 0 its straight
#         """
#         prev_xy = self.gps_to_xy(self.prev_pos[0], self.prev_pos[1])
#         curr_xy = self.gps_to_xy(self.curr_pos[0], self.curr_pos[1])

#         current_wp = self.mission_wp.waypoints[self.mission_wp.current_seq]
#         wp_xy = self.gps_to_xy(current_wp.x_lat, current_wp.y_long)

#         prev_vector = self.compute_vector(prev_xy, wp_xy)
#         curr_vector = self.compute_vector(curr_xy, wp_xy)
#         cross_product = np.cross(prev_vector, curr_vector)

#         cross_tol = 10

#         #if cross product is close to 0 then we know two vectors are collinear
#         if (abs(cross_product) <= cross_tol):
#             return False
#         else:
#             return True

#     def outside_geofence(self):
#         """"""
#         pass

#     def pre_no_go(self) -> boolean:
#         """return True if situation is no go"""
#         if not self.mission_wp.waypoints:
#             rospy.loginfo("no waypoints")
#             return True 
#         # if self.airspeed == None:
#         #     rospy.loginfo("airspeed is NONE")
#         #     return True
#         # if self.check_bad_distance(pti_duration_time):
#         #     return True
#         if self.check_curve_path():
#             rospy.loginfo("Curved path")
#             return True

#         return False

    
#     def run(self):
#         print("hello")
#         # self.no_go = self.pre_no_go()
#         self.values = self.global_position
#         rospy.spin()


class FlightCardListener():
    def __init__(self):
        self.port = PORT 
        self.max_connections = MAXCONNECTIONS
        self.correct_msg_len = 5
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

        pti_dict = {"FTI_MODE": fti_mode_val,
                    "FTI_FS_DURATION": time_duration,
                    "FTI_INJXN_POINT": injection_point,
                    "FTI_FS_AMP_BEGIN": amp_setting,
                    "FTI_FS_AMP_END": amp_setting,
                    "FTI_ENABLE": int(1) }

        return pti_dict

def close_socket(socket:socket):
    print("closing socket", socket)
    socket.close()

if __name__=='__main__':

    rospy.init_node("flight_card_listener", anonymous=False)
    rate_val = 10
    rate = rospy.Rate(rate_val)
    
    PORT = 9876
    MAXCONNECTIONS = 2
    THIS_IP = "10.3.7.205"
    BUFFSIZE = 1024

    listensocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    listensocket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    listensocket.bind(('', PORT))

    #open server
    listensocket.listen(MAXCONNECTIONS)

    OLD_ENABLE_VAL = None

    mavros.set_namespace()
    fc_listener = FlightCardListener()
    pti_verify = PTI.PTIParamVerifier()
    waypoint_observe = WaypointObserver.WaypointObserver()

    while not rospy.is_shutdown(): 
        clientsocket = None 

        try:      
            clientsocket,clientaddress = listensocket.accept()
            message_packet = clientsocket.recv(BUFFSIZE).decode()
            clientsocket.setblocking(0)
            message = message_packet.split()


            if fc_listener.is_msg_correct(message):

                #check if we SHOULD do this test based on prechecks
                
                good_message = message
                print("good message", good_message)
                pti_dict = fc_listener.parse_msg(good_message)

                if waypoint_observe.pre_no_go(pti_dict["FTI_FS_DURATION"]) == False:
                    
                    pti_verify.set_injection_vals("FTI_INJXN_POINT", 
                                pti_dict["FTI_INJXN_POINT"], pti_dict["FTI_FS_AMP_BEGIN"])

                    pti_verify.set_pti_param("FTI_MODE", pti_dict["FTI_MODE"])
                    pti_verify.set_pti_param("FTI_FS_DURATION", pti_dict["FTI_FS_DURATION"])
                    print(pti_dict["FTI_ENABLE"])
                    pti_verify.set_pti_param("FTI_ENABLE", pti_dict["FTI_ENABLE"])
                
                else:
                    print("no go")

            elif "END" in message:
                print("test is ending", message)
                pti_verify.set_pti_param("FTI_ENABLE", 0)
            
            else:
                print("bad inputs")

            print("\n---------------------")

            rate.sleep()
            

        except KeyboardInterrupt:
            if clientsocket:
                clientsocket.close()
            break


        
