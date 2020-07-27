#!/usr/bin/env python
# -*- coding: utf-8 -*-


import argparse
import time

import rospy
from dronekit import connect, VehicleMode, LocationGlobal
import rospy
from pymavlink import mavutil

from math import pi as PI

#from nav_msg.msg import Path
from quadrotor_msgs.msg import PositionCommand


TAKEOFF_OFFSET = 1
NODE_NAME = 'planner_executer'
POS_CMD_TOPIC_NAME = '/planning/pos_cmd'

parser = argparse.ArgumentParser(description='Planner commands execution')
parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used")
args = parser.parse_args()

connection_string = args.connect
sitl = None

#Start STIL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def arm_and_takeoff(aTargetAltitude):
    """
    Arms vehicle and takeoff to aTargetAltitude
    """

    print("Basic pre-arm checks")
    # Don't try to arm until autopilot is ready
    while not  vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED mode
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take Off target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    # (otherwise the command after vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: " , vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)


def pos_cmd_calibrate(pos_cmd):
    pos_x = pos_cmd.position.x
    pos_y = pos_cmd.position.y * -1
    pos_z = pos_cmd.position.z * -1 

    vel_x = pos_cmd.velocity.x
    vel_y = pos_cmd.velocity.y * -1
    vel_z = pos_cmd.velocity.z * -1

    yaw = 2 * PI - pos_cmd.yaw
    yaw_rate = pos_cmd.yaw_dot * -1

    return (
        float(pos_x),
        float(pos_y),
        float(pos_z),
        float(vel_x),
        float(vel_y),
        float(vel_z),
        float(yaw),
        float(yaw_rate),
        )

def execute_pos_cmd(pos_cmd):
    pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, yaw, yaw_rate = pos_cmd_calibrate(pos_cmd)

    frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                  # time_boot_ms (not used)
        0, 0,               # target system, target component
        frame,              # frame
        0b0000001111000000, # type_mask (only speeds enabled)
        pos_x,                  # X Positio
        pos_y,                  # Y Position 
        pos_z,                  # alt - Altitude in meters
        vel_x,                  # X velocity in NED frame in m/s
        vel_y,                  # Y velocity in NED frame in m/s
        vel_z,                  # Z velocity in NED frame in m/s
        0, 0, 0,                # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
        yaw, yaw_rate)          # yaw, yaw_rate

    vehicle.send_mavlink(msg)
    vehicle.flush()


def planner_listener():
    rospy.init_node(NODE_NAME, anonymous=False)
    sub = rospy.Subscriber(POS_CMD_TOPIC_NAME, PositionCommand, execute_pos_cmd)
    rospy.spin()


def main():
    rospy.loginfo("Listening to position commands from Planner Algorithm. ")

    arm_and_takeoff(TAKEOFF_OFFSET)
    planner_listener()

if __name__ == '__main__':
    main()
