#!/usr/bin/env python

import rospy
import math
import numpy as np
from tf.transformations import *
from std_msgs.msg import String


class Trigger:

    NODE_NAME="planner_trigger"
    TRIGGER_TOPIC="trigger"
    QUEUE_SIZE=1
    RATE_HZ=100

    def __init__(self):
        rospy.init_node(self.NODE_NAME, anonymous=False)
        self.pub = rospy.Publisher(self.TRIGGER_TOPIC, String, queue_size=self.QUEUE_SIZE)
        self.pub_rate = rospy.Rate(self.RATE_HZ)
        rospy.loginfo("<<Planner Trigger>>: node: %s, output_topic: /%s" % (self.NODE_NAME, self.TRIGGER_TOPIC))

    def trigger(self):
        while not rospy.is_shutdown():
            self.pub.publish("NOW!")
            rospy.loginfo("NOW!")
            self.pub_rate.sleep()


if __name__ == '__main__':
    try:
        trigger = Trigger()
        trigger.trigger()

    except rospy.ROSInterruptException:
        pass

