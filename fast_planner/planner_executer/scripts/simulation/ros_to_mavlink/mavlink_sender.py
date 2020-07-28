import os
import sys
import inspect

# Add current dir to sys.path
current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, current_dir)

import numpy as np
from dronekit import connect
import time
import threading



class MavlinkSender:

    def __init__(self):

        # Default configurations for connection to the FCU
        self.connection_string = '127.0.0.1:14550'
        self.connection_baudrate = 921600
        self.connection_timeout_sec = 5


        # FCU connection variables
        self.vehicle = None
        self.is_vehicle_connected = False

        # lock for thread synchronization
        self.lock = threading.Lock()


    def vehicle_connect(self):
        """
        Connects to ardupilot
        :return: True if connected , False if connection faile
        """

        try:
            self.vehicle = connect(self.connection_string, wait_ready=True, baud=self.connection_baudrate, source_system=1)
        except:
            print('Connection error! Retrying...')
            time.sleep(1)

        if self.vehicle == None:
            self.is_vehicle_connected = False
            return False
        else:
            self.is_vehicle_connected = True
            return True
        
    
    def send_vision_position_estimate_message(self, vision_sensor):
        """
        Gets ardupilot and send the data to ardupilot
        :param vision_sensor: odometry data that needs to be send to ardupilot
        :return:
        """
        
        with self.lock:
            if  vision_sensor.position is not None:

                # Setup covariance data, which is the upper right triangle of the covariance matrix, see here: https://files.gitter.im/ArduPilot/VisionProjects/1DpU/image.png

                covariance  = np.array([1, 0, 0, 0, 0, 0,
                                           1, 0, 0, 0, 0,
                                              1, 0, 0, 0,
                                                1, 0, 0,
                                                   1, 0,
                                                      1])

                current_time_us = int(round(time.time() * 1000000))


                # rpy_rad = tf.euler_from_quaternion(rpy_rad, 'sxyz')
                # Setup the message to be sent
                msg = self.vehicle.message_factory.vision_position_estimate_encode(
                    current_time_us,            # us Timestamp (UNIX time or time since system boot)
                    vision_sensor.ned_pos_x,   # Global X position
                    vision_sensor.ned_pos_y,   # Global Y position
                    vision_sensor.ned_pos_z,   # Global Z position
                    vision_sensor.roll,	                # Roll angle
                    vision_sensor.pitch,	                # Pitch angle
                    vision_sensor.yaw	                # Yaw angle
                    #covariance,                 # Row-major representation of pose 6x6 cross-covariance matrix
                    # vision_sensor.reset_counter               # Estimate reset counter. Increment every time pose estimate jumps.
                )

                print "pose :\n x: %f\n y: %f\n z: %f\n" %(vision_sensor.position.x, vision_sensor.position.y, vision_sensor.position.z)
                # print "orientation :\n x: %f\n y: %f\n z: %f\n w: %f\n" %(vision_sensor.quat[1], vision_sensor.quat[2], vision_sensor.quat[3], vision_sensor.quat[0])

                time.sleep(vision_sensor.latency / 1000.0)
                self.vehicle.send_mavlink(msg)
                self.vehicle.flush()


    # This func is not used

    def send_msg_to_gcs(self, text_to_be_sent):
        # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
        # Defined here: https://mavlink.io/en/messages/common.html#MAV_SEVERITY
        # MAV_SEVERITY = 3 will let the message be displayed on Mission Planner HUD, but 6 is ok for QGroundControl
        if self.is_vehicle_connected == True:
            text_msg =  text_to_be_sent
            # status_msg = self.vehicle.message_factory.statustext_encode(
            #     6,                      # MAV_SEVERITY
            #     text_msg.encode()	    # max size is char[50]
            # )
            # self.vehicle.send_mavlink(status_msg)
            # self.vehicle.flush()
            print("INFO: " + text_to_be_sent)
        else:
            print("INFO: Vehicle not connected. Cannot send text message to Ground Control Station (GCS)")
