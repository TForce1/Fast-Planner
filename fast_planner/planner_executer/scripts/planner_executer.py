#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import time

import rospy
from dronekit import connect
import rospy
from pymavlink import mavutil

from math import pi as PI

from quadrotor_msgs.msg import PositionCommand
from simulation.flight_command import fly, fly_gps
from simulation.math_utils.coordinates_utils import ENUtoNEDBodyFrame

TAKEOFF_OFFSET = 1
NODE_NAME = 'planner_executer'
POS_CMD_TOPIC_NAME = '/planning/pos_cmd'

parser = argparse.ArgumentParser(description='Planner commands execution')
parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used")
parser.add_argument('--gps', help="Is vehicle's ardupilot configured with or without GPS. If not specified, GPS is set to False", default=False)
args = parser.parse_args()

connection_string = args.connect
gps_available = args.gps
sitl = None

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def pos_cmd_calibrate(pos_cmd):
    pos_x, pos_y, pos_z = ENUtoNEDBodyFrame(pos_cmd.position.x, pos_cmd.position.y, pos_cmd.position.z)
    vel_x, vel_y, vel_z = ENUtoNEDBodyFrame(pos_cmd.velocity.x, pos_cmd.velocity.y, pos_cmd.velocity.z)
    acc_x, acc_y, acc_z = ENUtoNEDBodyFrame(pos_cmd.acceleration.x, pos_cmd.acceleration.y, pos_cmd.acceleration.z)

    yaw = 2 * PI - pos_cmd.yaw
    yaw_rate = pos_cmd.yaw_dot * -1

    return (
        float(pos_x),
        float(pos_y),
        float(pos_z),
        float(vel_x),
        float(vel_y),
        float(vel_z),
        float(acc_x),
        float(acc_y),
        float(acc_z),
        float(yaw),
        float(yaw_rate),
        )


def execute_pos_cmd(pos_cmd):
    print("Got pos_cmd\n")
    pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, yaw, yaw_rate = pos_cmd_calibrate(pos_cmd)
    # vel_y, vel_z = [0, 0]
    # pos_z = -TAKEOFF_OFFSET
    # vel_x = 10

    frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,                     # time_boot_ms (not used)
        0, 0,                  # target system, target component
        frame,		       # frame
        0b0000001111000000,    # type_mask (only acceleration disabled)
        pos_x, pos_y, pos_z,   # Position in (m)
        vel_x, vel_y, vel_z,   # Velocity in NED frame in (m/s)
        0, 0, 0,               # Acceleration (not supported yet, ignored in GCS_Mavlink)
        yaw,yaw_rate)

    print(msg)
    vehicle.send_mavlink(msg)
    vehicle.flush()


def planner_listener():
    rospy.init_node(NODE_NAME, anonymous=False)
    sub = rospy.Subscriber(POS_CMD_TOPIC_NAME, PositionCommand, execute_pos_cmd)

    rospy.spin()


def main():
    rospy.loginfo("Listening to position commands from Planner Algorithm. ")

    if gps_available:
        print("Taking off with GPS")
        fly_gps(vehicle, TAKEOFF_OFFSET)
    else:
        print("Taking off with NO GPS")
        fly(vehicle, TAKEOFF_OFFSET)
    planner_listener()

if __name__ == '__main__':
    main()

