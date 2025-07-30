"""world_base_link_tf_pub controller."""

# You may need to import some classes of the controller module. Ex:
# from controller import Robot, Motor, DistanceSensor
from numpy import matrix
import rospy
import tf
from controller import Supervisor
from scipy.spatial.transform import Rotation

# create the Robot instance.
supervisor = Supervisor()
robot_node = supervisor.getFromDef("CARBOT")

rospy.init_node(name="world_base_link_tf_pub", anonymous=True)

# get the time step of the current world.
timestep = int(supervisor.getBasicTimeStep())

br = tf.TransformBroadcaster()

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while supervisor.step(timestep) != -1 and not rospy.is_shutdown():

    pos = robot_node.getPosition()

    rot_matrix = robot_node.getOrientation()
    rot_matrix = [
        rot_matrix[0:3],
        rot_matrix[3:6],
        rot_matrix[6:10],
    ]
    
    r = Rotation.from_matrix(rot_matrix)

    quat = r.as_quat()
    
    br.sendTransform(
            translation=(pos[0], pos[1], pos[2]),
            rotation=quat,
            time=rospy.Time.now(),
            child="base_link",
            parent="world",
        )

