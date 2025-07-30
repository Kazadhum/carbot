#!/usr/bin/env python3

"""imu controller."""

from geometry_msgs.msg import Quaternion, Vector3
from webots_ros.srv import get_float, set_int
import rospy
from sensor_msgs.msg import Imu


class ImuDataMerger:
    def __init__(self):
        # Rospy Service Proxies
        rospy.wait_for_service(service="/robot/get_basic_time_step")
        robot_get_basic_time_step_srv = rospy.ServiceProxy(
            name="/robot/get_basic_time_step", service_class=get_float
        )
        rospy.wait_for_service(service="/accelerometer/enable")
        accelerometer_enable_srv = rospy.ServiceProxy(
            name="/accelerometer/enable", service_class=set_int
        )
        rospy.wait_for_service(service="/gyro/enable")
        gyro_enable_srv = rospy.ServiceProxy(name="/gyro/enable", service_class=set_int)

        # get the time step of the current world.
        timestep: int = int(robot_get_basic_time_step_srv().value)

        # Enable sensors
        accelerometer_enable_srv(value=timestep)
        gyro_enable_srv(value=timestep)

        rospy.init_node(name="imu_data_handler", anonymous=True)

        gyro_sub = rospy.Subscriber(
            name="/gyro/values", data_class=Imu, callback=self.gyro_recv
        )
        accel_sub = rospy.Subscriber(
            name="/accelerometer/values", data_class=Imu, callback=self.accel_recv
        )

        self.imu_pub = rospy.Publisher(name="imu", data_class=Imu, queue_size=10)

        self.gyro_msg = None
        self.accel_msg = None

        self.rate = rospy.Rate(hz=200)
        self.run()

    def gyro_recv(self, msg):
        self.gyro_msg = msg

    def accel_recv(self, msg):
        self.accel_msg = msg

    def run(self):
        while not rospy.is_shutdown():
            imu_msg = Imu()
            if self.gyro_msg and self.accel_msg:
                # Header
                imu_msg.header.stamp = self.accel_msg.header.stamp
                imu_msg.header.frame_id = "accelerometer"
                # Orientation (ignore)
                imu_msg.orientation = Quaternion(0, 0, 0, 1)
                imu_msg.orientation_covariance = [0] * 9
                imu_msg.orientation_covariance[0] = -1  # Ignore orientation
                # Linear Acceleration
                imu_msg.linear_acceleration = self.accel_msg.linear_acceleration
                # Angular Velocity
                imu_msg.angular_velocity = self.gyro_msg.angular_velocity
                self.imu_pub.publish(imu_msg)

            self.rate.sleep()


if __name__ == "__main__":
    merger = ImuDataMerger()
