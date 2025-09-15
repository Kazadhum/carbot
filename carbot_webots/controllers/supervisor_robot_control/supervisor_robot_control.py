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

# robot_controller= RobotController()

# Get wheels
wheel_names: list[str] = ["wheel" + str(i) for i in range(1, 5)]

service_dict = {}
# # Wait for ROS services to set velocity in the wheels

for wheel_name in wheel_names:
    rospy.wait_for_service(service=f"/{wheel_name}/set_position")
    rospy.wait_for_service(service=f"/{wheel_name}/set_velocity")
    rospy.wait_for_service(service=f"/{wheel_name}/set_torque")

    service_dict[f"{wheel_name}_set_position"] = rospy.ServiceProxy(
        name=f"/{wheel_name}/set_position",
        service_class=set_float,
    )

    service_dict[f"{wheel_name}_set_velocity"] = rospy.ServiceProxy(
        name=f"/{wheel_name}/set_velocity",
        service_class=set_float,
    )

    service_dict[f"{wheel_name}_set_torque"] = rospy.ServiceProxy(
        name=f"/{wheel_name}/set_torque",
        service_class=set_float,
    )

    # Set inicial values
    service_dict[f"{wheel_name}_set_position"](value=9999999999)
    service_dict[f"{wheel_name}_set_velocity"](value=0)
    service_dict[f"{wheel_name}_set_torque"](value=0)

# Main loop:
while supervisor.step(timestep) != -1 and not rospy.is_shutdown():
    
    t = supervisor.getTime()  # Convert the elapsed time from milisecs to secs

    if t > 1 and t < 1.5:
        trq = [0.01, 0.01]
    elif t > 1.5 and t < 4:
        for wheel_name in [wheel_names[0], wheel_names[2]]:
            service_dict[f"{wheel_name}_set_velocity"](value=0.1)
        for wheel_name in [wheel_names[1], wheel_names[3]]:
            service_dict[f"{wheel_name}_set_velocity"](value=0)
        continue

    elif t > 4 and t < 6:
        trq = [0.02, 0.02]
    else:
        trq = [0, 0]

    for wheel_name in [wheel_names[0], wheel_names[2]]:
    # for wheel_name in [wheel_names[0]]:
        service_dict[f"{wheel_name}_set_torque"](value=trq[0])

    for wheel_name in [wheel_names[1], wheel_names[3]]:
    # for wheel_name in [wheel_names[1]]:
        service_dict[f"{wheel_name}_set_torque"](value=trq[1])
