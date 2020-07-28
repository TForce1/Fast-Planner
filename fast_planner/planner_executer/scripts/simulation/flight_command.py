import os
import sys
import inspect

# Add current dir to sys.path
current_dir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
sys.path.insert(0, current_dir)

import time
import math
import dronekit
from pymavlink import mavutil
import argparse

from gazebo_sensor.vision_sensor import VisionSensor
from copter_utils.Home import set_home
from copter_utils.Takeoff import set_takeoff_thrust
from copter_utils.Position import set_position_local_ned
from copter_utils.Attitude import set_yaw


def fly(vehicle, aTargetAltitude):
   
    # Wait for the copter to be ready
    print("Basic pre-arm checks")
    while vehicle.mode.name == "INITIALISING":
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)
    time.sleep(3)

    # Set home location
    print("Setting home location")
    while not vehicle.home_location:
        set_home(vehicle)
        time.sleep(1)
    print("Home set to {}".format(vehicle.home_location))

    # Arm the copter
    print("Arming motors")
    set_mode(vehicle, "GUIDED_NOGPS")
    arm(vehicle)

    # Takeoff and set guided mode
    print("Taking off!")
    while True:
        if set_takeoff_thrust(vehicle, aTargetAltitude):
            break
        time.sleep(0.2)
    set_mode(vehicle, "GUIDED")

def connect(connection_string='127.0.0.1:14551'):
    vehicle = dronekit.connect(connection_string, wait_ready = True)
    return vehicle

def set_mode(vehicle, desired_mode):
    while vehicle.mode != desired_mode:
        vehicle.mode = dronekit.VehicleMode(desired_mode)
        time.sleep(0.2)
    print("mode is {}".format(vehicle.mode))

def arm(vehicle):
    vehicle.armed = True
    while not vehicle.armed:
        print(" Waiting for arming...")
        vehicle.armed = True
        time.sleep(1)
        print(vehicle.armed)


