#!/usr/bin/env python
from xmlrpc.client import Boolean

import rospy
import mavros
import numpy as np

from mavros import param

# from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from mavros_msgs.msg import FlightTestInput
# from mavros_msgs.msg import ParamValue
# from mavros_msgs.srv import ParamSet

from afrl_configs import pti_config, attitude_constraints

from supervisor import WaypointObserver
from tf.transformations import euler_from_quaternion, quaternion_from_euler

"""
- be able to set and get params without taking forever

rosrun mavros mavparam dump - gets all params 

rosrun mavros mavparam set FTI_ENABLE 1

safe-technology 
https://www.horizonhobby.com/blog-safe-technology-introduction.html

"""

# class Observer():
#     """Class KinematicObserver is in charge of observing data and checks the limit thresholds"""
#     def __init__(self, limit_range) -> None:
#         self.limit_range = limit_range

#     def outside_limits(self) -> Boolean:
#         """return True if outside, false if not"""


class AttitudeObserver():
    """Inherits from Observer class and observers Attitude position and rates of aircraft 
    based on odometry information"""
    def __init__(self, topic_name) -> None:
        #super().__init__(limit_range)
        
        self.rpy_radians = [None, None, None]  
        self.rpy_rates = [None, None, None]
        self.odom_sub = rospy.Subscriber(topic_name, Odometry, self.odom_cb)

    def odom_cb(self, msg):
        """callback for odometry data"""
        orientation_q = msg.pose.pose.orientation
        rpy_rate = msg.twist.twist.angular
        self.rpy_rates = [rpy_rate.x, rpy_rate.y, rpy_rate.z] 

        #orientation
        orientation_list = [orientation_q.x, orientation_q.y,
                             orientation_q.z, orientation_q.w]
        self.rpy_radians = euler_from_quaternion(orientation_list)
        

    def outside_limits(self,current_val, min_max_bounds) -> Boolean:
        """check if outside limits"""
        # print(min_max_bounds)
        #if current_val<= min_max_bounds[0] or current_val >= min_max_bounds[1]:
        if current_val <= -min_max_bounds or current_val>= min_max_bounds:
            return True
        else:
            return False

    def check_empty_states(self):
        """return True if no data is being recorded from states"""
        if self.rpy_radians == [None, None, None]:
            return True
        else:
            return False

    def inter_no_go(self):
        """check state and state rates, if outside then return True as in no go
        otherwise return False, need to log what is going on if there is a no_go"""
        
        #check if I have information on my states 
        if self.check_empty_states() == True:
            return True

        max_roll = param.param_get("SC_ROLL_ANG_MAX")
        max_pitch = param.param_get("SC_PITCH_ANG_MAX")

        max_roll_rate = param.param_get("SC_ROLL_RAT_MAX")
        max_pitch_rate = param.param_get("SC_PITCH_RAT_MAX")

        # print("params are", max_roll, max_pitch, max_roll_rate, max_pitch_rate)
        #check roll
        if self.outside_limits(self.rpy_radians[0], np.deg2rad(max_roll)):
            # rospy.loginfo("beyond roll threshold: {0:.9f}".
            # format(self.rpy_radians[0]))

            return True
        
        #check pitch
        if self.outside_limits(self.rpy_radians[1], np.deg2rad(max_pitch)):
            # rospy.loginfo("beyond pitch threshold: {0:.9f}".
            # format(self.rpy_radians[1]))

            return True

        #check roll rate 
        if self.outside_limits(self.rpy_rates[0], np.deg2rad(max_roll_rate)):
            # rospy.loginfo("beyond roll rate threshold: {0:.9f}".
            # format(self.rpy_rates[0]))
            return True

        #check pitch rate
        if self.outside_limits(self.rpy_rates[0], np.deg2rad(max_pitch_rate)):
            # rospy.loginfo("beyond pitch rate threshold: {0:.9f}".
            # format(self.rpy_rates[1]))
            return True

        return False   
    
    def print_info(self):
        """print info """
        if self.rpy_radians != [None, None, None]:
            print("rpy in deg is", np.rad2deg(self.rpy_radians))
            print("rpy RATES in deg/s is", np.rad2deg(self.rpy_rates)) 


class PTIParamVerifier():
    """Class FTIParamVerifier makes sure the parameters submitted are within the limits
    of the system"""

    def set_pti_param(self, pti_param: str , pti_param_val):
        """checks if pti param exists, not duplicate,
         values are good, and then sets value"""

        if self.check_pti_param_exist(pti_param) == False:
            print("doesn't exist", pti_param)
            return 

        if self.check_dup_param_val(pti_param, pti_param_val) == True:
            print("duplicate value")
            return 

        pti_param_range = pti_config.PTI_CONFIG[pti_param]
 
        #set amplitudes 
        if pti_param == "FTI_FS_AMP_BEGIN" or pti_param == "FTI_FS_AMP_END":
            if pti_param_val in pti_config.input_control_index:
                setting_vals = pti_config.FTI_INJXN_POINT[pti_param_val]
                if setting_vals == None:
                    return 
                
                control_index = pti_config.input_control_index[pti_param_val]
                param.param_set(pti_param, setting_vals[control_index])
                return 
            
            else:
                return 

        #set frequency
        if pti_param_val<= pti_param_range[1] and pti_param_val>=pti_param_range[0]:
            param.param_set(pti_param, pti_param_val)
            print("Set PTI", pti_param, "to", pti_param_val)
            return 
        else:
            print("value is out of range", pti_param_val, 
            "range of values are ", pti_param_range)
            return 

    def check_dup_param_val(self, pti_param: str, pti_param_val) -> bool:
        """check if parameter value is dup/same"""
        pti_val = self.get_pti_params(pti_param)
        if pti_param_val == pti_val:
            return True
        else:
            return False

    def check_pti_param_exist(self, pti_param: str) -> bool: 
        """check if pti param exists return True if so"""
        if pti_param in pti_config.PTI_CONFIG:
            return True 
        else:
            return False

    def get_pti_params(self, pti_param:str):
        """get pti params (str)"""
        if self.check_pti_param_exist(pti_param):
            pti_val = param.param_get(pti_param)
            # print("param value is of ", pti_param, " is ", pti_val)
            return pti_val
        else:
            print("hell naw", pti_param)
            return None

class SuperVisor():
    """interface controller that has observers and param verifirs to flag if operation is
    safe or not"""
    def __init__(self):
        pti_sub = rospy.Subscriber("mavros/flight_test_input",
                                    FlightTestInput,
                                    self.pti_cb)

        self.pti = PTIParamVerifier()
        self.att_obs = AttitudeObserver("mavros/odometry/in")
        self.wp_obs = WaypointObserver.WaypointObserver()
        self.pti_state = 0
        

    def pti_cb(self,msg):
        self.pti_state = msg.fti_state

    def get_params(self):
        pass

    def input_params(self):
        pass

    def any_pre_no_gos(self):
        """check for any prechecks for no gos such as stupid inputs
        or cant do the mission"""
        pti_dur_time = self.pti.get_pti_params("FTI_FS_DURATION")

        self.pre_no_go_list = [
            self.wp_obs.pre_no_go(pti_dur_time)
            ]
        
        if any(self.pre_no_go_list )== True:
            return True
        else:
            return False

    def any_inter_no_gos(self):
        """intermediate no go checks during pti operations"""
        self.inter_no_go_list = [
            # self.wp_obs.inter_no_go(),
            self.att_obs.inter_no_go()
            ]
        
        if any(self.inter_no_go_list) == True:
            return True
        else:
            return False

if __name__=='__main__':

    #initiate your node here 
    rospy.init_node("afrl_param_node", anonymous=False)
    
    rate_val = 50
    rate = rospy.Rate(rate_val)
    mavros.set_namespace()

    supervise = SuperVisor()
    print_inter_once = False

    #wait for params
    rospy.sleep(2.5)

    while not rospy.is_shutdown():    
        pti_enable = supervise.pti.get_pti_params("FTI_ENABLE")
        
        if pti_enable == 1 and supervise.any_inter_no_gos() == False:

            while not rospy.is_shutdown():
                if print_inter_once == False:
                    # print("Beginning PTI") 
                    print_inter_once = True
                    
                if supervise.pti_state == 4:
                    # print("test was a success!")
                    old_enable_val = 1 
                    print_inter_once = False
                    break

                if supervise.any_inter_no_gos() == True:
                    print("intermediate no go")
                    supervise.pti.set_pti_param("FTI_ENABLE" , 0)
                    print_inter_once = False
                    break

                rate.sleep()

        rate.sleep()




