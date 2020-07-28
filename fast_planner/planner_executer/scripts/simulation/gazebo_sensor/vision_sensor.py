import time

from gazebo_copter import GazeboSensor
from math_utils.coordinates_utils import to_ypr, ENUtoNEDBodyFrame

class VisionSensor:
    """
    A sensor sending vision position messages to the given vehicle,
    based on its "ground-truth" position obatianed from the Gazebo sensor.
    """
    def __init__(self, vehicle):
        """
        Create a new vision sensor.

        Parameters
        ----------
        vehicle
            A vehicle object obtained by dronekit.Connect()
        """
        self.gazebo_sensor = GazeboSensor()
        self.vehicle = vehicle

    def send_vision_position_message(self):
        """
        Send a vision position message to the vehicle associated with
        this sensor
        """
        state = self.gazebo_sensor.get_state()
        
        # Convert from Gazebo ENU to Arducopter NED
        position = state.position
        orientation = state.attitude.q
        (ned_pos_x, ned_pos_y, ned_pos_z) = ENUtoNEDBodyFrame(position.x, position.y, position.z)
        (ned_x, ned_y, ned_z) = ENUtoNEDBodyFrame(orientation['x'], orientation['y'], orientation['z'])
        q = {}
        q['x'] = ned_x
        q['y'] = ned_y
        q['z'] = ned_z
        q['w'] = orientation['w']
        [yaw, pitch, roll] = to_ypr(q)

        # Send vision message
        current_time_us = round(int(time.time()) * 1e6)
        msg = self.vehicle.message_factory.vision_position_estimate_encode(
            current_time_us,    # Timestamp (UNIX time or time since system boot)
            ned_pos_x,	        # Global X position
            ned_pos_y,          # Global Y position
            ned_pos_z,	        # Global Z position
            roll,	        # Roll angle
            pitch,	        # Pitch angle
            yaw	                # Yaw angle
        )
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()
