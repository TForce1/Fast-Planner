import math
from math import sin, cos, pi

import rospy
# import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from guassian_noise import add_guassian_noise
from gazebo_model import Position, Attitude, GazeboModel
from gazebo_copter import GazeboSensor
# from math_utils.coordinates_utils import to_ypr, ENUtoNEDBodyFrame

STANDARD_DEVIATION = 1


# sensor = GazeboSensor()
# model_state = sensor.get_state()
# position = model_state.position
# attitude = model_state.attitude


rospy.init_node('odometry_publisher')

odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
# odom_broadcaster = tf.TransformBroadcaster()

# x = 0.0
# y = 0.0
# z = 0.0
# roll = 0.0
# pitch = 0.0
# yaw = 0.0
#
# vx = 0.0
# vy = 0.0
# vz = 0.0
# vroll = 0.0
# vpitch = 0.0
# vyaw = 0.0


def set_message_params(position=None, attitude=None):

    if not position and not attitude:
        position = Position(0, 0, 0, 0, 0, 0)
        attitude = Attitude({'x': 0, 'y': 0, 'z': 0, 'w': 1}, 0, 0, 0)

    linear_pose = [position.x, position.y, position.z]
    linear_twist = [position.vx, position.vy, position.vz]
    quat = [attitude.q['x'], attitude.q['y'], attitude.q['z'], attitude.q['w']]
    angular_twist = [attitude.wx, attitude.wy, attitude.wz]

    noisy_linear_pose = add_guassian_noise(linear_pose, STANDARD_DEVIATION)
    noisy_quat = add_guassian_noise(quat, STANDARD_DEVIATION)
    norm_quat = [float(i)/sum(noisy_quat) for i in noisy_quat]
    noisy_linear_twist = add_guassian_noise(linear_twist, STANDARD_DEVIATION)
    noisy_angular_twist = add_guassian_noise(angular_twist, STANDARD_DEVIATION)


    return [noisy_linear_pose, norm_quat, noisy_linear_twist, noisy_angular_twist]


def send_nav_odom(state):

    r = rospy.Rate(1.0)
    count = 0
    while not rospy.is_shutdown():
        current_time = rospy.Time.now()

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        # odom_quat = tf.transformations.quaternion_from_euler(*noisy_angular_pose)

        ## first, we'll publish the transform over tf
        # odom_broadcaster.sendTransform(
        #     (state[0]),
        #     state[1],
        #     current_time,
        #     "base_link",
        #     "odom"
        # )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(*state[0]), Quaternion(*state[1]))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(*state[2]), Vector3(*state[3]))

        # publish the message
        odom_pub.publish(odom)

        print count ,current_time ,"\n"
        print "linear pose: " ,state[0] ,"\n"
        print "quaternion: " ,state[1] ,"\n"
        print "angles: " ,state[2] ,"\n"
        print "v angle: " ,state[3] ,"\n"
        print "======================="  ,"\n\n"

        print "pose: " , odom.twist.twist, "\n\n"


        count += 1
        r.sleep()


def main():

    state = set_message_params()
    send_nav_odom(state)


if __name__ == "__main__":

    main()
