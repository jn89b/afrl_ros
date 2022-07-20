#!/usr/bin/env python
import rospy 
import mavros 

"""
- Send waypoints to system for x amount of loops 
- If FTI_ENABLE is 1 and not a repeat of FTI_ENABLE 
see if we can do the PTI manuever:
    - check if collinear 
    - check if enough distance to track 
    - if good then do it 
    - if not then wait till loop back and begin the PTI 

"""


class MissionPlan():
    def __init__(self) -> None:
        pass

    

if __name__=='__main__':
    pass