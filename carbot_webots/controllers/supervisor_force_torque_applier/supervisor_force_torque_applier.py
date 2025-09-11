from subprocess import call
import numpy
from std_msgs.msg import Bool
from controller import Supervisor
import rospy
from barbot_bringup.msg import ApplyForce, ApplyTorque


class ForceApplierSupervisor:
    def __init__(self):

        # Webots Supervisor setup
        self.supervisor = Supervisor()
        self.robot_node = self.supervisor.getFromDef("CARBOT")
        self.time_step = int(self.supervisor.getBasicTimeStep())

        # ROS setup
        rospy.init_node("supervisor_force_node", anonymous=True)
        self.force_triggered = False
        self.torque_triggered = False
        self.slow_stop_triggered = False

        self.force_start_time = None
        self.torque_start_time = None

        # To block movement
        self.is_moving = True

        rospy.Subscriber("/apply_force", ApplyForce, self.force_callback)

        rospy.Subscriber("/apply_torque", ApplyTorque, self.torque_callback)

        rospy.Subscriber(
            name="/apply_slow_stop", data_class=Bool, callback=self.slow_stop_callback
        )

    def force_callback(self, msg):
        if not self.force_triggered:
            self.force_triggered = True
            self.force_start_time = self.supervisor.getTime()
            self.force_duration = msg.duration
            self.force_vector = [msg.force.x, msg.force.y, msg.force.z]

    def torque_callback(self, msg):
        if not self.torque_triggered:
            self.torque_triggered = True
            self.torque_start_time = self.supervisor.getTime()
            self.torque_duration = msg.duration
            self.torque_vector = [msg.torque.x, msg.torque.y, msg.torque.z]

    def slow_stop_callback(self, msg):
        if not self.slow_stop_triggered:
            self.slow_stop_triggered = True

        # while (self.robot_node.getVelocity() != [0.0]*6 and self.supervisor.step(self.time_step) != -1 and not rospy.is_shutdown()):
        #     velocities = self.robot_node.getVelocity()

        #     linear_vels = velocities[0:3]
        #     angular_vels = velocities[3:6]

        #     self.force_vector = [0.0] * 3
        #     self.torque_vector = [0.0] * 3

        #     for i in range(3):
        #         if linear_vels[i] > 0:
        #             self.force_vector[i] = -0.5
        #         elif linear_vels[i] < 0:
        #             self.force_vector[i] = 0.5

        #         if angular_vels[i] > 0:
        #             self.torque_vector[i] = -0.3
        #         elif angular_vels[i] < 0:
        #             self.torque_vector[i] = 0.3

        #     if numpy.linalg.norm(velocities) < 0.01:
        #         self.robot_node.setVelocity([0]*6)
        #         self.slow_stop_triggered = False
        #     else:
        #         self.robot_node.addForce(self.force_vector, relative=0)
        #         self.robot_node.addTorque(self.torque_vector, relative=0)

    def run(self):
        while self.supervisor.step(self.time_step) != -1 and not rospy.is_shutdown():

            current_time = self.supervisor.getTime()

            velocities = self.robot_node.getVelocity()
            linear_vels = velocities[0:3]
            angular_vels = velocities[3:6]

            if self.force_triggered:
                if current_time - self.force_start_time <= self.force_duration:
                    self.robot_node.addForce(self.force_vector, relative=0)
                else:
                    self.force_triggered = False

            if self.torque_triggered:
                if current_time - self.torque_start_time <= self.torque_duration:
                    self.robot_node.addTorque(self.torque_vector, relative=0)
                else:
                    self.torque_triggered = False


            if self.slow_stop_triggered:

                if self.force_triggered or self.torque_triggered:
                    continue

                self.force_vector = [0.0] * 3
                self.torque_vector = [0.0] * 3

                for i in range(3):
                    if linear_vels[i] > 0:
                        self.force_vector[i] = -0.5
                    elif linear_vels[i] < 0:
                        self.force_vector[i] = 0.5

                    if angular_vels[i] > 0:
                        self.torque_vector[i] = -0.3
                    elif angular_vels[i] < 0:
                        self.torque_vector[i] = 0.3

                # Force 0 vel if close enough to it
                if numpy.linalg.norm(velocities) < 0.01:
                    self.robot_node.setVelocity([0] * 6)
                    self.slow_stop_triggered = False
                else:
                    self.robot_node.addForce(self.force_vector, relative=0)
                    self.robot_node.addTorque(self.torque_vector, relative=0)


if __name__ == "__main__":
    supervisor = ForceApplierSupervisor()
    supervisor.run()
