import dronekit
from pymavlink import mavutil

def set_position_local_ned(vehicle, x, y, z):
    """
    Set the vehicle position to the given coordinates in a local NED
    coordinate system.

    Parameters
    ----------
    x : number
        The desired x coordinate, positive north
    y : number
        The desired y coordinate, positive east
    z : nuber
        The desired z coordiante, positive downward
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,
        0, 0,
        mavutil.mavlink.MAV_FRAME_BODY_NED,
        0b0000111111111000, # Consider only the position
            x, y, z,        #-- Position
            0, 0, 0,        #-- Velocity
            0, 0, 0,        #-- Accelerations
            0, 0)
    vehicle.send_mavlink(msg)
    vehicle.flush()

