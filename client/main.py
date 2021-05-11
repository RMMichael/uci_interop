import threading
from __future__ import print_function
from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative,Command
from pymavlink import mavutil # Needed for command message definitions
import time


# 