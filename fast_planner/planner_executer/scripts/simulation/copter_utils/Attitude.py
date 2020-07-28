import dronekit
from pymavlink import mavutil

def set_yaw(vehicle, heading, relative=False):
    """
    Set the yaw angle of the vehicle

    Parameters
    ----------
    vehicle : object
        The vehicle object, obtained from dronekit.connect()
    heading : number
        The desired yaw angle (in degrees)
    relative : boolean 
        Indicates if the yaw angle is relative to the current or absolute.
        Default: False
    """
    if relative:
        is_relative = 1 #yaw relative to direction of travel
    else:
        is_relative = 0 #yaw is an absolute angle

    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_CMD_CONDITION_YAW, #command
        0, #confirmation
        heading,    # param 1, yaw in degrees
        0,          # param 2, yaw speed deg/s
        1,          # param 3, direction -1 ccw, 1 cw
        is_relative, # param 4, relative offset 1, absolute angle 0
        0, 0, 0)    # param 5 ~ 7 not used
    vehicle.send_mavlink(msg)
    vehicle.flush()

