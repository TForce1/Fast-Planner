class GazeboModel:
    """ 
        A class representing a model in the Gazebo.
        Available fields are:
            position
                An object of type Position holding the model position
            attitude
                An object of type Attitude holding the model attitue
            name
                The name of the model (string)
    """
    def __init__(self, name, position, attitude):
        self.name = name
        self.position = position
        self.attitude = attitude

class Position:
    """ 
        A class representing a position of a model in the Gazebo.
        Available fields are:
            x
                The x coordinate
            y
                The y coordinate
            z
                The z coordinate
            vx
                The linear velocity in the x-axis
            vy
                The linear velocity in the y-axis
            vz
                The linear velocity in the z-axis
    """
    def __init__(self, x, y, z, vx, vy, vz):
        self.x = x
        self.y = y
        self.z = z
        self.vx = vx
        self.vy = vy
        self.vz = vz

class Attitude:
    """ 
        A class representing an attitude of a model in the Gazebo.
        Available fields are:
            q
                The quarternion of (x, y, z, w).
                A dict with the keys 'x', 'y', 'z', 'w'.
            wx
                The angular velocity in the x-axis
            wy
                The angular velocity in the y-axis
            wz
                The angular velocity in the z-axis
    """
 
    def __init__(self, q, wx, wy, wz):
        self.q = q
        self.wx = wx
        self.wy = wy
        self.wz = wz

