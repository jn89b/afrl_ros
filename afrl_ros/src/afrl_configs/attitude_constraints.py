# MIN and MAX of bounds
import numpy as np

## Inputs are in degrees but converted to radians 

##### ATTITUDE CONSTRAINTS ##### 
ROLL_VAL = 125
ROLL_BOUND = [np.deg2rad(-ROLL_VAL), np.deg2rad(ROLL_VAL)]

PITCH_VAL = 125
PITCH_BOUND = [np.deg2rad(-PITCH_VAL), np.deg2rad(PITCH_VAL)]

##### ATTITUDE RATE CONSTRAINTS #####
ROLL_R_VAL = 150 
ROLL_R_BOUND = [np.deg2rad(-ROLL_R_VAL), np.deg2rad(ROLL_R_VAL)]

PITCH_R_VAL = 150 
PITCH_R_BOUND = [np.deg2rad(-PITCH_R_VAL), np.deg2rad(PITCH_R_VAL)]

##### COMMAND CONSTRAINTS #####

