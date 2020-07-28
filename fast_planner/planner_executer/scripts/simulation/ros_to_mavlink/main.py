#! /usr/bin/env python

import sys
from mavlink_sender import MavlinkSender
from ros_odom_subscriber import Vision
import argparse

args_parser = argparse.ArgumentParser()
args_parser.add_argument('--topic', help='Desired ros topic to subscribe to. Default is \'\odom\'', type=str, default='/odom')
args_parser.add_argument('--rate', help='Desired rate to send mavlink in Hz', type=int, default=0)
args_parser.add_argument('--latency', help='Desired latency to send mavlink in mSec', type=int, default=0)
args = args_parser.parse_args()

topic_name = sys.argv[1]

sender = MavlinkSender()

print("INFO: Connecting to vehicle.")

while not sender.is_vehicle_connected:
    print "is vehicle connected: ", sender.is_vehicle_connected
    sender.is_vehicle_connected = sender.vehicle_connect()
print("INFO: Vehicle connected.")

vision_sensor = Vision(sender, args.topic, args.rate, args.latency)
vision_sensor.subscriber()

