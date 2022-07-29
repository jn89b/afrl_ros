#!/usr/bin/env python
from xmlrpc.client import Boolean
import rospy 
import math 
import numpy as np

from nav_msgs.msg import Odometry
from mavros_msgs.msg import  HomePosition, WaypointList
from sensor_msgs.msg import NavSatFix
# from scipy import spatial

"""
Check if on straight path

Check if about to land 

Check if out of bounds of geofence 

"""

class WaypointObserver():
    def __init__(self) -> None:

        self.home_position = HomePosition()
        self.global_position = NavSatFix()
        self.mission_wp = WaypointList()
        

        #SUBSCRIBERS
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback)

        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)

        self.mission_wp_sub = rospy.Subscriber('mavros/mission/waypoints', 
                                                WaypointList, 
                                                self.mission_wp_callback)

        self.airspeed_sub = rospy.Subscriber('mavros/odometry/in',
                                            Odometry, 
                                            self.airspeed_cb)
        """"""
        self.airspeed = None
        self.prev_pos = [None, None]
        self.curr_pos = [None, None]

    def check_vals_exist(self):
        """pass"""
        pass

    def home_position_callback(self, data):
        self.home_position = data

    def global_position_callback(self, data):
        """get gps data"""
        self.global_position = data

        #set this to current and previous positions
        
        if self.curr_pos == [None, None]:
            self.curr_pos = [data.latitude, data.longitude]
        else:
            self.prev_pos = self.curr_pos
            self.curr_pos = [data.latitude, data.longitude]


    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))

        self.mission_wp = data

    def airspeed_cb(self,data): 
        """get body airspeed of system"""
        self.airspeed = data.twist.twist.linear.x


    def distance_to_wp(self, lat:float, lon:float, alt:float):
        """alt(amsl): meters"""
        R = 6371000  # metres
        rlat1 = math.radians(lat)
        rlat2 = math.radians(self.global_position.latitude)

        rlat_d = math.radians(self.global_position.latitude - lat)
        rlon_d = math.radians(self.global_position.longitude - lon)

        #haversine equation
        a = (math.sin(rlat_d / 2) * math.sin(rlat_d / 2) + math.cos(rlat1) *
             math.cos(rlat2) * math.sin(rlon_d / 2) * math.sin(rlon_d / 2))
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        d = R * c
        # alt_d = abs(alt - self.altitude.amsl)

        #rospy.logdebug("d: {0}, alt_d: {1}".format(d))#, alt_d))
        return d

    def check_bad_distance(self, pti_duration_time:float) -> Boolean:
        """check if possible to do the pti during the duration, by 
        multiplying the current set velocity by the time to get the distance, 
        see how much distance is left to the next waypoint, if greater than we cant do it"""
        
        lat_dist_desired = self.airspeed * pti_duration_time
        current_wp = self.mission_wp.waypoints[self.mission_wp.current_seq]
        remain_lat_d = self.distance_to_wp(current_wp.x_lat, 
                                            current_wp.y_long,
                                            current_wp.z_alt) 
        # print("lat desired is ", lat_dist_desired, "remaining", remain_lat_d)
        if abs(lat_dist_desired) >= abs(remain_lat_d):
            # rospy.loginfo("too close to current distance dist_remaining: {0:.9f},{0:.9f},{0:.9f}".
            # format(remain_lat_d), (lat_dist_desired), (remain_lat_d))
            return True
        else:
            return False

    def compute_vector(self, point_1, point_2):
        """given two points compute the vector"""
        some_vec = np.array(point_2)- np.array(point_1)
        return some_vec

    def gps_to_xy(self, lat:float, lon:float):
        """converts gps coordinates to x y coordinates in meters"""
        R = 6371000  # metres
        rlat = math.radians(lat)
        rlon = math.radians(lon)
        # rlat2 = math.radians(self.global_position.latitude)
        x = R * math.cos(rlat) * math.cos(rlon)
        y = R * math.cos(rlat) * math.sin(rlon)

        return [x,y]

    def check_curve_path(self):
        """
        check if vector directions are not straight returns True if so
        if I take the dot product between two vectors and get 1 then its straight
        if I take the cross product between two vectors and get 0 its straight
        """
        prev_xy = self.gps_to_xy(self.prev_pos[0], self.prev_pos[1])
        curr_xy = self.gps_to_xy(self.curr_pos[0], self.curr_pos[1])

        current_wp = self.mission_wp.waypoints[self.mission_wp.current_seq]
        wp_xy = self.gps_to_xy(current_wp.x_lat, current_wp.y_long)

        prev_vector = self.compute_vector(prev_xy, wp_xy)
        curr_vector = self.compute_vector(curr_xy, wp_xy)
        cross_product = np.cross(prev_vector, curr_vector)
        cross_tol = 25 #deg tolerance

        #if cross product is close to 0 then we know two vectors are collinear
        if (abs(cross_product) <= cross_tol):
            return False
        else:
            return True

    def outside_geofence(self):
        """"""
        pass

    def pre_no_go(self, pti_duration_time:float) -> Boolean:
        """return True if situation is no go"""
        if not self.mission_wp.waypoints:
            rospy.loginfo("no waypoints")
            return True 
        if self.airspeed == None:
            rospy.loginfo("airspeed is NONE")
            return True
        if self.check_bad_distance(pti_duration_time):
            rospy.loginfo("bad distance")
            return True
        if self.check_curve_path():
            rospy.loginfo("Curved path")
            return True

        return False


    def inter_no_go(self) -> Boolean:
        """intermediate no gos for waypoint checker"""
        if self.check_curve_path():
            rospy.loginfo("Curved path")
            return True


        return False



