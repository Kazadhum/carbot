#!/usr/bin/env python3

""" A simple controller for barbot"""

from logging import RootLogger
from pprint import pprint
import random
import time

import numpy as np
import rospy
from webots_ros.msg import Float64Stamped
from webots_ros.srv import get_float, set_float, set_int
from robotController import RobotController


def get_time(t0):
    return (rospy.Time.now() - t0).to_sec()


def get_vel_vec(controller: RobotController):
    vels = controller.get_velocity()

    vel_vec = [
        vels.linear.x,
        vels.linear.y,
        vels.linear.z,
        vels.angular.x,
        vels.angular.y,
        vels.angular.z,
    ]

    return vel_vec


def main():
    """Main control loop."""
    controller = RobotController()

    # Raise it up first
    controller.slow_stop()
    
    
    controller.add_force(force=[7, 0, 0], duration=2)
    rospy.sleep(duration=2)
    # controller.slow_stop()
    
    rospy.spin()


if __name__ == "__main__":
    main()
