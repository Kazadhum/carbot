"""supervisor_robot_control controller."""

from __future__ import annotations

import rospy
from webots_ros.srv import set_float
from controller import Supervisor

# create the Robot instance.
supervisor = Supervisor()
robot = supervisor.getFromDef("CARBOT")
timestep = int(supervisor.getBasicTimeStep())
rospy.init_node(name="supervisor_robot_control", anonymous=True)

# Get wheels
wheel_names: list[str] = ["wheel" + str(i) for i in range(1, 5)]

service_dict = {}
# Wait for ROS services to set velocity in the wheels

for wheel_name in wheel_names:
    rospy.wait_for_service(service=f"/{wheel_name}/set_position")
    rospy.wait_for_service(service=f"/{wheel_name}/set_velocity")

    service_dict[f"{wheel_name}_set_position"] = rospy.ServiceProxy(
        name=f"/{wheel_name}/set_position",
        service_class=set_float,
    )

    service_dict[f"{wheel_name}_set_velocity"] = rospy.ServiceProxy(
        name=f"/{wheel_name}/set_velocity",
        service_class=set_float,
    )

    # Set inicial values
    service_dict[f"{wheel_name}_set_position"](value=9999999999)
    service_dict[f"{wheel_name}_set_velocity"](value=0.1)

# Main loop:
while supervisor.step(timestep) != -1 and not rospy.is_shutdown():

    t = supervisor.getTime()  # Convert the elapsed time from milisecs to secs

    if t < 1:
        vel = [0, 0]
    elif t > 1 and t < 10:
        vel = [5, 5]
    elif t > 10 and t < 20:
        vel = [1, 5]
    elif t > 20 and t < 30:
        vel = [0, 4]
    elif t > 30:
        vel = [0,0]

    for wheel_name in [wheel_names[0], wheel_names[2]]:
        service_dict[f"{wheel_name}_set_velocity"](value=vel[0])

    for wheel_name in [wheel_names[1], wheel_names[3]]:
        service_dict[f"{wheel_name}_set_velocity"](value=vel[1])
