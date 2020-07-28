import rospy

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import GetPhysicsProperties, GetWorldProperties, GetModelState
from gazebo_physics import GazeboPhysics, Gravity
from gazebo_model import Position, Attitude, GazeboModel

class GazeboSensor:
    """ 
        A sensor sampled the copter data from the Gazebo.
    """
    
    def __init__(self):
        """Create a new instance of GazeboSensor."""
        self.handler = GazeboCopterHandler()

    def get_state(self):
        """Get a GazeboModel object representing the state of the copter
           
           Returns
           -------
           GazeboModel
                An object holding the current state of the copter
        """
        return self.handler.get_state()

class GazeboCopterHandler:
    """
        A class responsible for grabbing information from the Gazebo.
        Mainly for internal use by GazeboSensor.
    """

    def __init__(self):
        """Create a new instance of GazeboCopterHandler"""
        self._attach()

    def _attach(self):
        """Find the name of the iris model in the Gazebo.
           NOTE: Currently we assume only one copter, so we find the first model with the
                 string 'iris' in its name.
        """
        self.model_name = None
        rospy.wait_for_service('/gazebo/get_world_properties')
        try:
            world_properties_service = rospy.ServiceProxy('/gazebo/get_world_properties', GetWorldProperties)
            world_properties = world_properties_service()
            for model in world_properties.model_names:
                if 'iris' in model:
                    self.model_name = model
                    break
        except rospy.ServiceException as e:
            print(e)

        if not self.model_name:
            raise Excpetion("Can't find iris model")
        self.model_name.replace('-', '_')
        print("Found model {0}".format(self.model_name))

    def get_state(self):
        """Get a GazeboModel object representing the state of the copter
           
           Returns
           -------
           GazeboModel
                An object holding the current state of the copter
        """
        rospy.wait_for_service('/gazebo/get_model_state')
        try:
            model_state_service = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
            model_state = model_state_service(self.model_name, '')
            return GazeboModel(
                    self.model_name,
                    Position(model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z,
                        model_state.twist.linear.x, model_state.twist.linear.y, model_state.twist.linear.z),
                    Attitude({'x': model_state.pose.orientation.x, 'y': model_state.pose.orientation.y,
                        'z': model_state.pose.orientation.z, 'w': model_state.pose.orientation.w}, model_state.twist.angular.x,
                        model_state.twist.angular.y, model_state.twist.angular.z))
        except rospy.ServiceException as e:
            print(e)

    def get_physics_state(self):
        """Get a GazeboPhysics object representing the physics data of the simulation
           
           Returns
           -------
           GazeboPhysics
                An object holding the current physics state of the simulation
        """
        rospy.wait_for_service('/gazebo/get_physics_properties')
        try:
            physics_properties_service = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
            physics_properties = physics_properties_service()
            return GazeboPhysics(physics_properties.time_step, 
                    Gravity(physics_properties.gravity.x, physics_properties.gravity.y, physics_properties.gravity.z))
        except rospy.ServiceException as e:
            print(e)
        
