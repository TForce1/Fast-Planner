import dronekit
import math

from math_utils.coordinates_utils import to_quaternion

DEFAULT_TAKEOFF_THRUST = 0.55
SMOOTH_TAKEOFF_THRUST = 0.52

def set_takeoff_thrust(vehicle, aTargetAltitude):
    """
    Check if the vehicle has reached the target altitude. If not, set a thrust
    so it elevates. The thrust is relative to its current altitude with respect
    to the desired altitude.

    Parameters
    ----------
    vehicle : object
        A vehicle object obtained with dronekit.connect()
    aTargetAltitude : number
        The desired altitude of the vehicle

    Returns
    -------
    True if the vehicle has reached the target altitude, False otherwise.
    """
    thrust = DEFAULT_TAKEOFF_THRUST
    current_altitude = vehicle.location.global_relative_frame.alt
    if current_altitude >= aTargetAltitude * 0.95: # Trigger just below target alt.
        print("Reached target altitude")
        return True
    elif current_altitude >= aTargetAltitude * 0.6:
        thrust = SMOOTH_TAKEOFF_THRUST
    send_attitude_target(vehicle, thrust = thrust)
    return False

def send_attitude_target(vehicle, roll_angle = 0.0, pitch_angle = 0.0, 
        yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False, thrust = 0.5):

    if yaw_angle is None:
        # This value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw

    msg = vehicle.message_factory.set_attitude_target_encode(
            0, # time_boot_ms
            1, # Target system
            1, # Target component
            0b00000000 if use_yaw_rate else 0b00000100,
            to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
            0, # Body roll rate in radians
            0, # Body pitch rate in radians
            math.radians(yaw_rate), # Body yaw rate in radians/second
            thrust
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

