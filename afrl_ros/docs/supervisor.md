# Supervisor Job 
- Make sure parameter inputs are "good" and input params if so
- Make sure aircraft during flight is stable  

## Parameters
To set PTI param:
- Does param exist?  
- Is param value duplicate?
- Is param value within range?
- Set to high, medium, and low
If passes checks then we set params to flight controller 

## Supervisor Control 
- Need to listen to states of the aircraft ****
- Determine if instability will happen, if so then reject PTI protocol and return to system 

- Check battery, if within threshold return

### Airspeed check
- Check airspeed for possible stall speed characteristics:
    - Stall speed is min speed at which aircraft must fly to product lift, lift generated by angle of attack as air moves over wings. 
    - Want to be above this speed threshold otherwise aircraft can't fly
- Determining st

### Attitude check
- Check attitudes of system,  


### Flight Envelope check
- 400ft to 6000ft AGL

## Parameter inputs 
- Parameter for PTI are defined in the PX4 flight stack:
    https://github.com/jn89b/afrl_px4/blob/main/src/modules/fw_att_control/flight_test_input/flight_test_input_params.c
- The Supervisor makes sure that parameters set to the flight stack are for PTI ONLY which it does by checking the list in the src/afrl_ros/pti_config.py file
- In addition it will check the values for each of these configs are "correct" 
- We define correct as in:
    - Correct data structure type
    - Values are within a range or tolerance 