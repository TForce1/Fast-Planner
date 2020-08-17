#!/usr/bin/env python
# -*- coding: utf-8 -*-

import argparse
import time

import rospy
from dronekit import connect
import rospy
from pymavlink import mavutil

from math import pi as PI

from simulation.flight_command import fly
from simulation.math_utils.coordinates_utils import ENUtoNEDBodyFrame

from std_msgs.msg import String
from geometry_msgs.msg import Point

TAKEOFF_OFFSET = 1
NODE_NAME = 'dummy_fly_node'
POS_CMD_TOPIC_NAME = '/trigger'

parser = argparse.ArgumentParser(description='Planner commands execution')
parser.add_argument('--connect', help="Vehicle connection target string. If not specified, SITL automatically started and used")
args = parser.parse_args()

connection_string = args.connect
sitl = None

QUEUE_SIZE=1
SUBSCRIBE_RATE_HZ=30

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)


def pos_cmd_calibrate(pos_cmd):
    pos_x, pos_y, pos_z = ENUtoNEDBodyFrame(pos_cmd.position.x, pos_cmd.position.y, pos_cmd.position.z)
    vel_x, vel_y, vel_z = ENUtoNEDBodyFrame(pos_cmd.velocity.x, pos_cmd.velocity.y, pos_cmd.velocity.z)
    acc_x, acc_y, acc_z = ENUtoNEDBodyFrame(pos_cmd.acceleration.x, pos_cmd.acceleration.y, pos_cmd.acceleration.z)

    yaw = 2 * PI - pos_cmd.yaw
    yaw_rate = pos_cmd.yaw_dot * -1 
    return (float(pos_x), float(pos_y), float(pos_z),
            float(vel_x), float(vel_y), float(vel_z),
            float(acc_x), float(acc_y), float(acc_z),
            float(yaw), float(yaw_rate))


def execute_pos_cmd(pos_cmd):
    #pos_x, pos_y, pos_z, vel_x, vel_y, vel_z, acc_x, acc_y, acc_z, yaw, yaw_rate = pos_cmd_calibrate(pos_cmd)
    pos_x, pos_y, pos_z = ENUtoNEDBodyFrame(pos_cmd.x, pos_cmd.y, TAKEOFF_OFFSET)
    rospy.loginfo("(%d,%d,%d)" % (pos_x, pos_y, pos_z))

    vel_x, vel_y, vel_z = 0.5,0,0
    acc_x, acc_y, acc_z = 0,0,0
    yaw, yaw_rate = 0,0

    frame = mavutil.mavlink.MAV_FRAME_LOCAL_NED

    # create the SET_POSITION_TARGET_LOCAL_NED command
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
            0,                      # time_boot_ms (not used)
            0, 0,                   # target system, target component
            frame,                  # frame
            0b0000001111000111,     # type_mask (only speeds enabled)
            pos_x,                  # X Position
            pos_y,                  # Y Position
            pos_z,                  # alt - Altitude in meters
            vel_x,                  # X velocity in NED frame in m/s
            vel_y,                  # Y velocity in NED frame in m/s
            vel_z,                  # Z velocity in NED frame in m/s
            acc_x, acc_y, acc_z,    # afx, afy, afz acceleration (not supported yet, ignored in GCS_Mavlink)
            yaw, yaw_rate)          # yaw, yaw_rate

    vehicle.send_mavlink(msg)
    vehicle.flush()

    rospy.loginfo(msg)


def dummy_listener():
    rospy.init_node(NODE_NAME, anonymous=False)
    sub = rospy.Subscriber(POS_CMD_TOPIC_NAME, Point, execute_pos_cmd, queue_size=QUEUE_SIZE)
    rospy.spin()


def main():
    rospy.loginfo("Listening to dummy trihher")

    vehicle = connect(connection_string, wait_ready=True)
    fly(vehicle,TAKEOFF_OFFSET)
    dummy_listener()


if __name__ == '__main__':
    main()

