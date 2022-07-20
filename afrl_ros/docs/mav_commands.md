Refer to this document for MAVLINK common commands:
https://mavlink.io/en/messages/common.html


param set FW_AIRSPD_TRIM 20

When sending commands to waypoint.command the following values define the following protocols:
- waypoint.command = 22 -> MAV_CMD_NAV_TAKEOFF
- waypoint.command = 16 -> MAV_CMD_NAV_WAYPOINT
- waypoint.command = 31 -> MAV_CMD_NAV_LOITER_TO_ALT https://mavlink.io/en/messages/common.html#MAV_CMD_LOITER_TO_ALT
- waypoint.command = 189 -> MAV_CMD_DO_LAND_START
- waypoint.command = 21 -> MAV_CMD_NAV_LAND 

