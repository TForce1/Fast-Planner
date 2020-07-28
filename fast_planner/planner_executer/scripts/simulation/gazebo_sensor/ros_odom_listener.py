# import roslib; roslib.load_manifest('Phoebe')
import rospy
# import irobot_create_2_1

from dronekit import connect, VehicleMode
from time import sleep
from nav_msgs.msg import Odometry
from vision_to_mavlink import odom_to_mavlink, vehicle_connect
from geometry_msgs.msg import *
from tf.msg import *


connection_string = '127.0.0.1:14550'
connection_baudrate = 921600
connection_timeout_sec = 5
is_vehicle_connected = False



def callback(odom_data):

    # rospy.sleep(1)
    curr_time = odom_data.header.stamp

    print("INFO: Connecting to vehicle.")
    while (is_vehicle_connected == False):
        print "is vehicle connected: " ,is_vehicle_connected
        is_vehicle_connected = vehicle_connect()

    print("INFO: Vehicle connected.")
    odom_to_mavlink(odom_data.pose.pose, odom_data.twist.twist, curr_time)



# def transformation(tf_data):
#     global counter
#     rospy.sleep(1)
#     transform = tf_data.transform
#     print transform


def begin():


    print ("waiting for broadcast...")
    rospy.init_node('odometry', anonymous=True) #make node
    rospy.Subscriber('/odom',Odometry,callback, queue_size=1)


if __name__ == "__main__":
    begin()
    rospy.spin() # not really necessary because we have while not rospy.is_shutdown()
