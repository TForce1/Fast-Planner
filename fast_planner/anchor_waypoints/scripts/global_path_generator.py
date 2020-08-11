#!/usr/bin/env python

import rospy
import math
import time
import numpy as np
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Point
from geometry_msgs.msg import Pose

from tf.transformations import *

class GlobalPath:

    NODE_NAME="global_path"
    ODOMETRY_TOPIC="odom"
    WAYPOINTS_TOPIC="anchor_waypoints"
    QUEUE_SIZE=100
    SAFE_DISTANCE=3

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)
        self.sub = rospy.Subscriber(self.ODOMETRY_TOPIC, Odometry, self.transform)
        self.pub = rospy.Publisher(self.WAYPOINTS_TOPIC, PoseStamped, queue_size=self.QUEUE_SIZE)
        rospy.loginfo("<<Global Path>>: node: %s, input_topic: /%s, output_topic: /%s" % \
                      (self.NODE_NAME, self.ODOMETRY_TOPIC, self.WAYPOINTS_TOPIC))

        self.way_points = [[50,0,1],[100,0,1],[100,-50,1],[100,-100,1]]
        self.current_point = 0
        self.mission_accomplished = False

    @staticmethod
    def reached_position(safe_distance, position, point):
        distance = math.sqrt(((point[0] - position.x)**2)+((point[1] - position.y)**2))
        if distance <= safe_distance:
            print "reached the destination"
            return True
        return False

    def transform(self, msg):
        if self.mission_accomplished:
            return
        if (self.current_point == 0 or self.reached_position(self.SAFE_DISTANCE, msg.pose.pose.position, self.way_points[self.current_point-1])):
            dest_x, dest_y, dest_z = self.way_points[self.current_point]

            # PoseStamped Message
            point = Point(x=dest_x, y=dest_y, z=dest_z)
            position_msg = Pose(position=point, orientation=Quaternion(0,0,0,1))
            path_pose_stamped = PoseStamped(header=msg.header, pose=position_msg)

            time.sleep(1)
            self.pub.publish(path_pose_stamped)
            print("Send %d,%d,%d" % (dest_x, dest_y, dest_z))

            self.current_point += 1

            if (self.current_point == len(self.way_points)):
                print("MISSION ACCOMPLISHED!!!")
                self.mission_accomplished = True


if __name__ == '__main__':
    try:
        path = GlobalPath()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass

