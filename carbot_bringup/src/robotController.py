#!/usr/bin/env python3

from barbot_bringup.msg._ApplyForce import ApplyForce
from barbot_bringup.msg._ApplyTorque import ApplyTorque
from geometry_msgs.msg import Twist, Vector3
import rospy
import numpy as np
from std_msgs.msg import Bool
from webots_ros.srv._get_float import get_float
from webots_ros.srv._get_uint64 import get_uint64
from webots_ros.srv._node_add_force_or_torque import node_add_force_or_torque
from webots_ros.srv._node_get_pose import node_get_pose
from webots_ros.srv._node_get_velocity import node_get_velocity
from webots_ros.srv._node_set_velocity import node_set_velocity
from webots_ros.srv._supervisor_get_from_def import supervisor_get_from_def


class RobotController:
    def __init__(self) -> None:

        rospy.init_node(name="carbot_controller", anonymous=True)

        # Wait for necessary services
        services_to_wait_for = [
            "/supervisor/get_self",
            "/supervisor/node/add_force",
            "/supervisor/node/add_torque",
            "/supervisor/node/get_pose",
            "/supervisor/get_from_def",
        ]

        # Also wait for getBasicTimeStep service
        services_to_wait_for.append("/robot/get_basic_time_step")
        # Additional services
        services_to_wait_for.append("/robot/get_time")

        for srv in services_to_wait_for:
            rospy.wait_for_service(service=srv)

        robot_get_basic_time_step_srv = rospy.ServiceProxy(
            name="/robot/get_basic_time_step", service_class=get_float
        )

        supervisor_get_self_srv = rospy.ServiceProxy(
            name="/supervisor/get_self", service_class=get_uint64
        )

        supervisor_get_from_def_srv = rospy.ServiceProxy(
            name="/supervisor/get_from_def", service_class=supervisor_get_from_def
        )

        self.supervisor_node = int(supervisor_get_self_srv().value)

        self.basic_time_step = int(robot_get_basic_time_step_srv().value)

        self.world_node = supervisor_get_from_def_srv(name="WORLD_DUMMY", proto=0).node

        self.srv_proxy_dict = {
            "add_torque": rospy.ServiceProxy(
                name="/supervisor/node/add_torque",
                service_class=node_add_force_or_torque,
            ),
            "add_force": rospy.ServiceProxy(
                name="/supervisor/node/add_force",
                service_class=node_add_force_or_torque,
            ),
            "get_pose": rospy.ServiceProxy(
                name="/supervisor/node/get_pose", service_class=node_get_pose
            ),
            "get_velocity": rospy.ServiceProxy(
                name="/supervisor/node/get_velocity", service_class=node_get_velocity
            ),
            "set_velocity": rospy.ServiceProxy(
                name="/supervisor/node/set_velocity", service_class=node_set_velocity
            ),
        }

        self.apply_force_pub = rospy.Publisher(
            name="/apply_force", data_class=ApplyForce, queue_size=10, latch=True
        )
        self.apply_torque_pub = rospy.Publisher(
            name="/apply_torque", data_class=ApplyTorque, queue_size=10, latch=True
        )
        self.apply_slow_stop_pub = rospy.Publisher(
            name="/apply_slow_stop", data_class=Bool, queue_size=10, latch=True
        )

    def get_pose(self):
        return self.srv_proxy_dict["get_pose"](
            node=self.world_node, from_node=self.supervisor_node
        ).pose

    def add_torque(self, torque: list, duration: float):

        msg = ApplyTorque()
        msg.duration = duration
        msg.torque = Vector3()
        msg.torque.x, msg.torque.y, msg.torque.z = torque

        self.apply_torque_pub.publish(msg)

        # self.srv_proxy_dict["add_torque"](
        #     node=self.supervisor_node, force=torque_vec, relative=0
        # )

    def add_force(self, force: list, duration: float):

        msg = ApplyForce()
        msg.duration = duration
        msg.force = Vector3()
        msg.force.x, msg.force.y, msg.force.z = force

        self.apply_force_pub.publish(msg)

        # self.srv_proxy_dict["add_force"](
        #     node=self.supervisor_node, force=force_vec, relative=0
        # )

    def get_velocity(self):
        vel = self.srv_proxy_dict["get_velocity"](node=self.supervisor_node).velocity
        return vel

    def set_velocity(self, linear: list, angular: list):
        linear_vec = Vector3()
        linear_vec.x, linear_vec.y, linear_vec.z = linear

        angular_vec = Vector3()
        angular_vec.x, angular_vec.y, angular_vec.z = angular

        vel = Twist()
        vel.linear = linear_vec
        vel.angular = angular_vec

        self.srv_proxy_dict["set_velocity"](node=self.supervisor_node, velocity=vel)

    def stop_robot(self):
        self.set_velocity(linear=[0] * 3, angular=[0] * 3)

    def slow_stop(self):
        msg = Bool(data=True)
        self.apply_slow_stop_pub.publish(msg)

