"""plain_wheel_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
import rospy
from controller import Robot

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

wheels = []
wheelsNames = ['wheel1', 'wheel2', 'wheel3', 'wheel4']

for i in range(4):
    wheels.append(robot.getDevice(wheelsNames[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0.0)

wheelsL = [wheels[0], wheels[2]]
wheelsR = [wheels[1], wheels[3]]

velL = 0
velR = 0

print(wheels)
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1 and not rospy.is_shutdown():

    t = robot.getTime()

    if t < 3:
        velL = 0
        velR = 0
    elif t > 3 and t < 10:
        velL = 5
        velR  = 5
    elif t > 10 and t < 15:
        velL = 0
        velR = 8
    elif t > 15 and t < 20:
        velL = 8
        velR = 0
    elif t > 20 and t < 30:
        velL = 0
        velR = 3
    else:
        velL = 0
        velR = 0

    for wheel in wheelsL:
        wheel.setVelocity(velL)
    for wheel in wheelsR:
        wheel.setVelocity(velR)    


# Enter here exit cleanup code.
