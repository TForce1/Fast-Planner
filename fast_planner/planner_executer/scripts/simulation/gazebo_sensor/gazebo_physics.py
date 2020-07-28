class GazeboPhysics:
    """ 
        A class representing the physics state of the simulation in the Gazebo
        Available fields are:
            time_step
                The dt
            gravity
                An object of type Gravity holding gravity data
    """
    def __init__(self, time_step, gravity):
        self.time_step = time_step
        self.gravity = gravity

class Gravity:
    """ 
        A class representing the gravity forces of the simulation
        Available fields are:
            x
                The gravity force in the x axis
            y
                The gravity force in the y axis
            z
                The gravity force in the z axis
    """
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

