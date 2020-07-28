import os
import sys

# Add current dir to sys.path
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, current_dir)

import rospy
from nav_msgs.msg import Odometry
import numpy as np
import threading
import time
from pyquaternion import Quaternion
from math_utils.coordinates_utils import to_ypr, ENUtoNEDBodyFrame


class Vision:

    def __init__(self, sender, topic, rate, latency):

        # Sender is instance that is connected to ardupilot and sends to it Mavlink msgs
        self.sender = sender
        # topic name to sucbscribe to
        self.topic = topic
        # rate in Hz to send Mavlink msgs
        self.rate = rate
        # latency generator
        self.latency = latency

        self.last_sent = None

        self.prev_pose = None
        self.prev_twist = None

        # For AP, a non-zero "self.reset_counter" would mean that we could be sure that the user's setup was using mavlink2
        self.reset_counter = 1





    def calc_matrixes(self, pose, twist):
        """
        Gets odometry data , transforms it to coordinate system of ardupilot
        :param pose: position and orientation data of odometry
        :param twist: linear velocity and angular velocity data of odometry
        :return:
        """
        if pose:

            quat = Quaternion(pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z)
            #rot_quat = (0.5, 0.5, -0.5, -0.5)
            
            #quat = quat * rot_quat
            
            self.position = pose.position

            (self.ned_pos_x, self.ned_pos_y, self.ned_pos_z) = ENUtoNEDBodyFrame(pose.position.x, \
                                                                  pose.position.y, pose.position.z)

            (ned_x, ned_y, ned_z) = ENUtoNEDBodyFrame(quat[1], quat[2],
                                                      quat[3])

            q = {}
            q['x'] = ned_x
            q['y'] = ned_y
            q['z'] = ned_z
            q['w'] = quat[0]
            [self.yaw, self.pitch, self.roll] = to_ypr(q)


            # Check for pose jump and increment reset_counter
            if self.prev_pose != None and self.prev_twist != None:
                delta_translation = [pose.position.x - self.prev_pose.position.x, pose.position.y - self.prev_pose.position.y,
                                     pose.position.z - self.prev_pose.position.z]
                position_displacement = np.linalg.norm(delta_translation)

                # Pose jump is indicated when position changes abruptly. The behavior is not well documented yet (as of librealsense 2.34.0)
                jump_threshold = 1  # in meters, from trials and errors, should be relative to how frequent is the position data obtained (200Hz for the T265)
                if (position_displacement > jump_threshold):
                    # send_msg_to_gcs('Pose jump detected')
                    print("Position jumped by: ", position_displacement)
                    self.reset_counter += 1

            self.prev_pose = pose
            self.prev_twist = twist



    def callback(self, odom_data):
        """
        This func is called each time an Odometry type ros topic is sent.
        Sends to sender the data that needs to be send to ardupilot

        """
        # rospy.sleep(1)
        # curr_time = odom_data.header.stamp
        curr_time = int(round(time.time() * 1000000))

        self.calc_matrixes(odom_data.pose.pose, odom_data.twist.twist)

        # if rate is 0 - the rate of the msgs is the rate of received ros topic msgs
        if self.last_sent is None or self.rate == 0:
            self.last_sent = curr_time
            self.sender.send_vision_position_estimate_message(self)

        elif curr_time - self.last_sent >= 1000000 // self.rate:
            print "diff: ", (int(curr_time) - self.last_sent)
            self.last_sent = curr_time
            self.sender.send_vision_position_estimate_message(self)

        # self.sender.send_vision_speed_estimate_message(self)
        # print odom_data.pose
        # odom_to_mavlink(odom_data.pose.pose, odom_data.twist.twist, curr_time)


    def subscriber(self):

        print ("waiting for broadcast...")
        rospy.init_node('odometry', anonymous=True)  # make node
        rospy.Subscriber(self.topic, Odometry, self.callback, queue_size=1)
        rospy.spin()
