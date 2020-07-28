import math

def ENUtoNEDBodyFrame(x, y, z):
    """
    Convert (x, y, z) location in ENU coordinates to NED coordinates

    Parameters
    ----------
    x : number
        The value of the x coordinate of the location
    y : number
        The value of the y coordinate of the location
    z : number
        The value of the z coordinate of the location

    Returns
    -------
    (x, y, z) tuple in NED coordinates
    """
    return (x, -y, -z)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions

    Parameters
    ----------
    roll : number
        The roll angle (in degrees)
    pitch : number
        The pitch angle (in degrees)
    yaw : number
        The yaw angle (in degrees)

    Returns
    -------
    [w, x, y, z] - The quaternion corresponding to the given parameters
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))
    
    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5
    
    return [w, x, y, z]


def to_ypr(q):
    """
    Convert quaternions to roll,pitch,yaw

    Parameters
    ----------
    q : dict
        A dictionary with the keys 'w', 'x', 'y', 'z' holding
        quaternion values

    Returns
    -------
    [yaw, pitch, roll] - The angles (in degrees) corresponding to 
                         the given quaternion
    """
    x = q['x']
    y = q['y']
    z = q['z']
    w = q['w']
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

