# useful notes
# sim vehicle must be built before it works
# use --gimbal to simulate camera
# https://github.com/dronekit/dronekit-sitl
# use pip3 install pymavlink==2.4.8
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative,Command
from pymavlink import mavutil # Needed for command message definitions
import time
import math
from helpers import set_roi, get_vehicle_attributes, arm_and_takeoff, adds_square_mission, distance_to_current_waypoint, get_location_metres

import argparse  
parser = argparse.ArgumentParser()
parser.add_argument('--connect', 
                   help="Vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

connection_string = args.connect
sitl = None


#Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()


# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

get_vehicle_attributes(vehicle)



# Get Vehicle Home location - will be `None` until first set by autopilot
while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for home location ...")
# We have a home location, so print it!        
print("\n Home location: %s" % vehicle.home_location)

#Confirm current value on vehicle by re-downloading commands
cmds = vehicle.commands
cmds.download()
cmds.wait_ready()
print(" New Home Location (from vehicle - altitude should be 222): %s" % vehicle.home_location)

print("\nSet Vehicle.mode = GUIDED (currently: %s)" % vehicle.mode.name) 
vehicle.mode = VehicleMode("GUIDED")
while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
    print(" Waiting for mode change ...")
    time.sleep(5)
    # if vehicle.mode != 'GUIDED':
    #     vehicle.mode = VehicleMode("GUIDED")

adds_square_mission(vehicle, vehicle.location.global_frame, 50)

# Arm and take of to altitude of 5 meters
arm_and_takeoff(vehicle, 10)

print("Starting mission")
# Reset mission set to first (0) waypoint
vehicle.commands.next = 0

# Set mode to AUTO to start mission
vehicle.mode = VehicleMode("AUTO")
while not vehicle.mode.name=='AUTO':  #Wait until mode has changed
    print(" Waiting for mode change ...")
    time.sleep(1)
oglocation = vehicle.location.global_frame
# Monitor mission. 
# Demonstrates getting and setting the command number 
# Uses distance_to_current_waypoint(), a convenience function for finding the 
#   distance to the next waypoint.
last_height = 0

while True:
    nextwaypoint = vehicle.commands.next
    print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint(vehicle)))
    
    if nextwaypoint == 5:  # Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
        point1 = get_location_metres(oglocation, 50, -50)
        cmds.add(
            Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0,
                    0,
                    0, 0, 0, point1.lat, point1.lon, 11))
        print("added waypoint")
    if nextwaypoint == 6:
        
        print("Exit 'standard' mission when start heading to final waypoint (5)")
        break;
    time.sleep(1)

print('Return to launch')
vehicle.mode = VehicleMode("RETURN_TO_LAUNCH")
while not vehicle.mode.name=='RETURN_TO_LAUNCH':  #Wait until mode has changed
    print(" Waiting for mode change ...")
    time.sleep(1)

## Reset variables to sensible values.
print("\nReset vehicle attributes/parameters and exit")
vehicle.mode = VehicleMode("STABILIZE")
# vehicle.armed = False

# Close vehicle object before exiting script
print("\nClose vehicle object")
vehicle.close()

# Shut down simulator if it was started.
if sitl is not None:
    sitl.stop()

print("Completed")
