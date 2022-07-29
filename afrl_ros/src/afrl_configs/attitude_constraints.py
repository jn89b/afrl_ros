# MIN and MAX of bounds
import numpy as np

## Inputs are in degrees but converted to radians 

##### ATTITUDE CONSTRAINTS ##### 
ROLL_VAL = 25
ROLL_BOUND = [np.deg2rad(-ROLL_VAL), np.deg2rad(ROLL_VAL)]

PITCH_VAL = 25
PITCH_BOUND = [np.deg2rad(-PITCH_VAL), np.deg2rad(PITCH_VAL)]

##### ATTITUDE RATE CONSTRAINTS #####
ROLL_R_VAL = 100 
ROLL_R_BOUND = [np.deg2rad(-ROLL_R_VAL), np.deg2rad(ROLL_R_VAL)]

PITCH_R_VAL = 100 
PITCH_R_BOUND = [np.deg2rad(-PITCH_R_VAL), np.deg2rad(PITCH_R_VAL)]

##### COMMAND CONSTRAINTS #####

"""
testing was done with attitude limits at 25 degrees 

and attitude rates at 0.49 degrees per second

"""