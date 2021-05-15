import threading
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative,Command
from pymavlink import mavutil # Needed for command message definitions
import time
from dronekit.helpers import *


# shared data structure template
class Utilities:
    def __init__(self):
        self.collision_avoid = []
        self.isCollision = False 
        


if __name__ == '__main__':
    util = Utilities()
    curr_stage = 1
    last_waypoint = 0
    
    
    
    # Pathplanning, get formatted file
    # Connect to server and relay telemetry & pass util
    # start ob avoid thread & pass util
    
    while True:
        if util.isCollision:
            # execute avoid function
            print("avoid func")
        if curr_stage == 1:
            print("stage 1")
            
        elif curr_stage == 2:
            print("stage 2")
            
        elif curr_stage == 3:
            print("stage 3")
        
        
        
    