#!/usr/bin/env python3

import asyncio
import time
from mavsdk import System
import mavsdk.mission_raw
from mavsdk.mission import (MissionItem, MissionPlan)



async def run():
    drone = System()
    print("Waiting for drone to connect...")
    await drone.connect(system_address="udp://:14540")
    #await drone.connect()
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break

    #await print_mission_progress(drone)
    # await drone.mission.mission_progress()
    # await drone.mission.start_mission() 
    # await drone.mission.set_current_mission_item()
    
    # time.sleep(2)
    # await drone.action.hold()
    # print("-- Stopping")

    # mission_import_data = await drone.mission_raw.import_qgroundcontrol_mission("FW_mission_5.json")
    # print(f"{len(mission_import_data.mission_items)} mission items imported")
    # await drone.mission_raw.upload_mission(mission_import_data.mission_items)
    # print("Mission uploaded")

async def print_mission_progress(drone):
    async for mission_progress in drone.mission.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")


if __name__=='__main__':
    loop = asyncio.get_event_loop()
    loop.run_until_complete(run())