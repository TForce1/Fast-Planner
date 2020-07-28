import dronekit
from pymavlink import mavutil

HOME_LAT_DEFAULT = -35.363261
HOME_LON_DEFAULT = 149.165235
HOME_ALT_DEFAULT = 584.0

def set_home(vehicle, lat = HOME_LAT_DEFAULT, lon = HOME_LON_DEFAULT, alt = HOME_ALT_DEFAULT):
    """
    Set GPS origin and home position for the copter

    Parameters
    ----------
    lat : number
        The latitude coordinate of the home location
        Default value is HOME_LAT_DEFAULT
    lon : number
        The longtitude coordinate of the home location
        Default value is HOME_LON_DEFAULT
    alt:
        The altitude corrdinate of the home location
        Default value is HOME_ALT_DEFAULT
    """

    # Set the EKF origin
    msg = vehicle.message_factory.set_gps_global_origin_encode(
        0,
        lat * 1e7,
        lon * 1e7,
        alt * 1e3
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

    # Set the home position
    msg = vehicle.message_factory.command_long_encode(
        0, 0,  # target system, target component
        mavutil.mavlink.MAV_CMD_DO_SET_HOME,  # command
        0,  # confirmation
        0,  # param 1: 1 to use current position, 2 to use the entered values.
        0, 0, 0,  # params 2-4 : unused
        lat, lon, alt
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

