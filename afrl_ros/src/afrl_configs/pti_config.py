# PTI_CONFIG = ["FTI_ENABLE", "FTI_FS_AMP_BEGIN", "FTI_FS_AMP_END",
#                 "FTI_FS_DURATION", "FTI_FS_FRQ_BEGIN", "FTI_FS_FRQ_END",
#                 "FTI_FS_FRQ_RAMP", "FTI_FS_INJXN_POINT", "FTI_MODE"]

import numpy as np


PTI_CONFIG = {
    "FTI_MODE": [0, 1],
    "FTI_ENABLE": [0, 1],
    "FTI_INJXN_POINT": [0, 9], #actuators 1-3: r,p,y #rate 4-6: r,p,y #at 7-9: r,p,y
    "FTI_FS_DURATION": [1, 200], #seconds
    "FTI_FS_FRQ_BEGIN": [0.01, 50.0], #hz
    "FTI_FS_FRQ_END": [0.01, 100.0], #hz
    "FTI_FS_AMP_BEGIN": [0.0,250.0],  #deg/%
    "FTI_FS_AMP_END": [0.0,250.0], #deg/%
    "FTI_FS_FRQ_RAMP": [0.0, 10] #power  
}


#### LOW MEDIUM HIGH VALUES #########
input_control_index = {
    "Low":  0,
    "Medium":  1,
    "High": 2
}

control_settings = [0.1, 0.3, 0.5] #percent
rate_settings = [np.deg2rad(15), np.deg2rad(30), np.deg2rad(45)] #deg/s
attitude_settings = [np.deg2rad(5), np.deg2rad(10), np.deg2rad(20)] #deg

FTI_INJXN_POINT = {
    "0" : None,
    "1" : control_settings,
    "2" : control_settings,
    "3" : control_settings,
    "4" : rate_settings, 
    "5" : rate_settings,
    "6" : rate_settings,
    "7" : attitude_settings, 
    "8" : attitude_settings,
    "9" : attitude_settings,
}

LOOP_GAIN = {
    "BASE" : 1.0,
    "INCREASE": 1.2,
    "DECREASE" : 0.8 
}

