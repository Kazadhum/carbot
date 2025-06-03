#!/usr/bin/env python3

"""simple controller for carbot"""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from geometry_msgs.msg import Quaternion, Vector3
from sensor_msgs.msg._Imu import Imu


from rospy.topics import Publisher


from controller import Supervisor, Robot, Accelerometer, Gyro
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
import os
import tf
from rosgraph_msgs.msg import Clock

# create the Robot instance.
robot = Supervisor()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# Enable sensors
accelerometer = robot.getDevice('accelerometer')
accelerometer.enable(timestep)
gyroscope = robot.getDevice('gyroscope')
gyroscope.enable(timestep)

# Get wheels
wheels = []
wheel_names: "list[str]" = ['wheel' + str(i) for i in range(1,5)]

for i in range(len(wheel_names)):
    wheels.append(robot.getDevice(wheel_names[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.01)

# Create ROS publisher for sensor
rospy.init_node(name="data_handler", anonymous=True)
imu_pub: Publisher = rospy.Publisher(name="imu", data_class=Imu, queue_size=10)

# Now set up the publisher of the current pose of the robot (world-base_link tf)

robot_node = robot.getSelf()
robot_trans = robot_node.getField("translation")
robot_rot = robot_node.getField("rotation")

br = tf.TransformBroadcaster()

# Clock publisher
clock_pub = rospy.Publisher(name='clock', data_class=Clock, queue_size=1)

t = 0
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1 and not rospy.is_shutdown():
    
    t += timestep/1000.0 # Convert the elapsed time from milisecs to secs
    rospy.loginfo(msg="t = " + str(t))


    # Control
    is_turning: bool = False
    # if t < 1 or t > 10:
        # robot_node.setVelocity([0, 0, 0, 0, 0, 0])
    # elif t > 1 and t < 5:
        # robot_node.setVelocity([0, 0, 0, 0, 0, 1])
    # elif t > 5 and t < 10:
        # robot_node.setVelocity([0, 0, 0, 0, 0, -1])


    if t < 1:
        vel = [0, 0]
    if t > 1 and t < 5:
        vel = [2, 1]
    elif (t > 5 and t < 9):
        vel = [2, 0.5]
    elif t > 9:
        vel = [2, 3]
    for wheel in [wheels[0], wheels[2]]: # Left wheels
        wheel.setVelocity(vel[0])
    for wheel in [wheels[1], wheels[3]]: # Right wheels
        wheel.setVelocity(vel[1])

    wheels_l = [wheels[0], wheels[2]] 
    wheels_r = [wheels[1], wheels[3]]
    
    # Start standing still
    # if t < 1:
        # for wheel in wheels:
            # wheel.setTorque(0)
    # Accelerate for a second
    # elif t > 1 and t < 2:
        # for wheel in wheels:
            # wheel.setTorque(0.01)
    # Decelerate for 1 sec
    # elif t > 2 and t < 3:
        # for wheel in wheels:
            # wheel.setTorque(-0.01)
    # Maintain constant velocity
    # elif t > 2.5 and t < 4:
        # for wheel in wheels:
            # wheel.setTorque(0)
    # Rotate -> introduce difference in velocity
    # elif t > 4 and t < 6:
        # for wheel in wheels_l:
            # wheel.setVelocity(1)
        # for wheel in wheels_r:
            # wheel.setVelocity(0)

    # Read the sensors:    
    gyro_values = gyroscope.getValues()
    accel_values = accelerometer.getValues()
    
    # Build imu message
    msg: Imu = Imu()
    
    # Header
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = 'accelerometer'

    # Orientation (ignore)
    msg.orientation = Quaternion(0, 0, 0, 1)
    msg.orientation_covariance = [0] * 9
    msg.orientation_covariance[0] = -1 # Ignore orientation

    # Linear Acceleration
    msg.linear_acceleration = Vector3(*accel_values)
    msg.linear_acceleration_covariance = [0] * 9

    # Angular Velocity
    msg.angular_velocity = Vector3(*gyro_values)
    msg.angular_velocity_covariance = [0] * 9

    imu_pub.publish(msg)
    # rate.sleep()

    # publish world-base_link tf
    pos = robot_trans.getSFVec3f()
    rot = robot_rot.getSFRotation()

    # Convert axis-angle to quaternion
    quat = tf.transformations.quaternion_about_axis(rot[3], (rot[0], rot[1], rot[2]))

    br.sendTransform(
        (pos[0], pos[1], pos[2]),       # x, y, z
        quat,                           # Quaternion
        rospy.Time.now(),
        "base_link",
        "world"                          # or "map"
    )

    # Publish /clock
    sim_time = robot.getTime()
    secs = int(sim_time)
    nsecs = int((sim_time - secs) * 1e9)

    clock_msg = Clock()
    clock_msg.clock.secs = secs
    clock_msg.clock.nsecs = nsecs
    clock_pub.publish(clock_msg)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
