# JUNE 28th
- [x] Implement a class that can set PX4 parameters 
- [x] Check for params from user input based on a TCP port 


## Set params 
- Check if FTI_VALS exist
- If valid values or not stupid
- Check if FTI_ENABLE is already set 

Supervisor: 
- Listen for PTI inputs and check no duplicate
- Check if PTI inputs are not stupid 
- Set PTI inputs to system 
- Check attitude of aircraft 
- If aircraft is unstable, then end the PTI controller and notify the 


Kinematic Observer:
- Check if attitude, attitude rates, body position, body position rates are at some "stupid values", out of bounds
  - If so then return True 
  - If not return False
- 

