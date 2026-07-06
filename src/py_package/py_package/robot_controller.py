import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

from math import *
import numpy as np
import random

class RobotController(Node):
    def __init__(self):
        super().__init__("robot_controller")
        self._logger = self.get_logger()
        self._joint_states_publisher = self.create_publisher(JointState, "/joint_states", 5)
        self._setpoint_publisher = self.create_publisher(Marker, "/setpoint", 5)
        self._marker_publisher = self.create_publisher(Marker, "/marker", 5)

        self._joint_states = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        self._base_link_setpoint = np.array([1.0, 1.0, 1.0, 1.0])
        self._setpoint = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link"),
            ns="setpoint",
            id=0,
            type=Marker.ARROW,
            points=[Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=0.0, z=0.0)],
            scale=Vector3(x=0.05, y=0.1, z=0.0),
            color=ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0),
        )
        self._marker = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link"),
            ns="marker",
            id=0,
            type=Marker.ARROW,
            points=[Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=0.0, z=0.0)],
            scale=Vector3(x=0.05, y=0.1, z=0.0),
            color=ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0),
        )
        
        self._timer_period_sec = 0.02
        self.create_timer(self._timer_period_sec, self.robot_controller_callback)
    
    def forward(self, joint_states):
        state1, state2, state3, state4, state5, state6 = joint_states
        t1 = np.array(
            [
                [cos(state1), -sin(state1), 0.0, 0.0],
                [-sin(state1), -cos(state1), 0.0, 0.0],
                [0.0, 0.0, -1.0, 0.14],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        t2 = np.array(
            [
                [0.0, 0.0, -1.0, -0.0536635],
                [-sin(state2), -cos(state2), 0.0, 0.0],
                [-cos(state2), sin(state2), 0.0, -0.18],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        t3 = np.array(
            [
                [0.0, 0.0, -1.0, 0.825],
                [-sin(state3), -cos(state3), 0.0, 0.0],
                [-cos(state3), sin(state3), 0.0, -0.0536656],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        t4 = np.array(
            [
                [0.0, 0.0, -1.0, -0.0536635],
                [-sin(state4), -cos(state4), 0.0, 0.0],
                [-cos(state4), sin(state4), 0.0, -0.08],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        t5 = np.array(
            [
                [0.0, 0.0, -1.0, 0.825],
                [-sin(state5), -cos(state5), 0.0, 0.0],
                [-cos(state5), sin(state5), 0.0, -0.0536656],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        t6 = np.array(
            [
                [0.0, 0.0, -1.0, -0.0536635],
                [-sin(state6), -cos(state6), 0.0, 0.0],
                [-cos(state6), sin(state6), 0.0, -0.08],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        t7 = np.array(
            [
                [1.0, 0.0, 0.0, 0.425],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, -0.0536635],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )
        return t1 @ t2 @ t3 @ t4 @ t5 @ t6 @ t7 @ np.array([0.0, 0.0, 0.0, 1.0])

    def robot_controller_callback(self):
        now = self.get_clock().now().to_msg()

        self._setpoint = Marker(
            header=Header(stamp=now, frame_id="base_link"),
            ns="setpoint",
            id=0,
            type=Marker.ARROW,
            points=[
                Point(x=0.0, y=0.0, z=0.0),
                Point(
                    x=self._base_link_setpoint[0],
                    y=self._base_link_setpoint[1],
                    z=self._base_link_setpoint[2],
                ),
            ],
            scale=Vector3(x=0.05, y=0.1, z=0.0),
            color=ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0),
        )
        self._setpoint_publisher.publish(self._setpoint)

        end_effector_in_base_link = self.forward(self._joint_states)
        self._marker = Marker(
            header=Header(stamp=now, frame_id="base_link"),
            ns="marker",
            id=0,
            type=Marker.ARROW,
            points=[
                Point(x=0.0, y=0.0, z=0.0),
                Point(
                    x=end_effector_in_base_link[0],
                    y=end_effector_in_base_link[1],
                    z=end_effector_in_base_link[2],
                ),
            ],
            scale=Vector3(x=0.05, y=0.1, z=0.0),
            color=ColorRGBA(r=0.0, g=0.0, b=255.0, a=1.0),
        )
        self._marker_publisher.publish(self._marker)
        
        error = self._base_link_setpoint[:3] - end_effector_in_base_link[:3]
        if sqrt(error[0]**2 + error[1]**2 + error[2]**2) >= (epsilon := 0.01):
            jacobian = np.zeros((3, 6))
            for i in range(len(self._joint_states)):
                joint_states = [*self._joint_states]
                joint_states[i] += epsilon
                x, y, z, _ = self.forward(joint_states)
                jacobian[0][i] = (x - end_effector_in_base_link[0]) / epsilon
                jacobian[1][i] = (y - end_effector_in_base_link[1]) / epsilon
                jacobian[2][i] = (z - end_effector_in_base_link[2]) / epsilon
            states_change = np.linalg.pinv(jacobian) @ error
            self._joint_states += epsilon * states_change
        else:
            self._base_link_setpoint[random.randint(0, 1)] *= -1
            self._base_link_setpoint[2] = random.uniform(1.5, 2.0)

        jointState = JointState()
        jointState.header.stamp = now
        jointState.name = [
            "arm1_base",
            "arm1",
            "arm2_base",
            "arm2",
            "arm3_base",
            "arm3",
        ]
        jointState.position = [*self._joint_states]
        self._joint_states_publisher.publish(jointState)


def main():
    rclpy.init()
    rclpy.spin(RobotController())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
