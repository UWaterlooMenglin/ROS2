import rclpy
from rclpy.node import Node

from std_msgs.msg import Header, ColorRGBA
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker

from math import *
import numpy as np

class RobotController(Node):
    def __init__(self):
        super().__init__('robot_controller')
        self._logger = self.get_logger()
        self._joint_states_publisher = self.create_publisher(JointState, "/joint_states", 5)
        self._marker_publisher = self.create_publisher(Marker, "/marker", 5)
        self._marker = Marker(header=Header(stamp=self.get_clock().now().to_msg(), frame_id="base_link"),
                              ns="marker",
                              id=0,
                              type=Marker.ARROW,
                              points=[Point(x=0.0, y=0.0, z=0.0), Point(x=0.0, y=0.0, z=0.0)],
                              scale=Vector3(x=0.05, y=0.1, z=0.0),
                              color=ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0))
        self._base_link_setpoint = np.array([0.0, 0.0, 0.0, 1.0])
        self.create_timer(0.02, self.robot_controller_callback)

    def robot_controller_callback(self):
        jointState = JointState()
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        position = (pi / 2) * sin(((2 * pi) / 20) * now_sec)
        
        jointState.header.stamp = now.to_msg()
        jointState.name = ["arm1_base", "arm1", "arm2_base", "arm2", "arm3_base", "arm3"]
        jointState.position = [position, position, position, position, position, position]
        self._joint_states_publisher.publish(jointState)
        
        t1 = np.array([
            [cos(position),     -sin(position),     0.0,        0.0],
            [-sin(position),    -cos(position),     0.0,        0.0],
            [0.0,               0.0,                -1.0,       0.14],
            [0.0,               0.0,                0.0,        1.0]
        ])
        t2 = np.array([
            [0.0,               0.0,                -1.0,       -0.0536635],
            [-sin(position),    -cos(position),     0.0,        0.0],
            [-cos(position),    sin(position),      0.0,        -0.18],
            [0.0,               0.0,                0.0,        1.0]
        ])
        t3 = np.array([
            [0.0,               0.0,                -1.0,       0.825],
            [-sin(position),    -cos(position),     0.0,        0.0],
            [-cos(position),    sin(position),      0.0,        -0.0536656],
            [0.0,               0.0,                0.0,        1.0]
        ])
        t4 = np.array([
            [0.0,               0.0,                -1.0,       -0.0536635],
            [-sin(position),    -cos(position),     0.0,        0.0],
            [-cos(position),    sin(position),      0.0,        -0.08],
            [0.0,               0.0,                0.0,        1.0]
        ])
        t5 = np.array([
            [0.0,               0.0,                -1.0,       0.825],
            [-sin(position),    -cos(position),     0.0,        0.0],
            [-cos(position),    sin(position),      0.0,        -0.0536656],
            [0.0,               0.0,                0.0,        1.0]
        ])
        t6 = np.array([
            [0.0,               0.0,                -1.0,       -0.0536635],
            [-sin(position),    -cos(position),     0.0,        0.0],
            [-cos(position),    sin(position),      0.0,        -0.08],
            [0.0,               0.0,                0.0,        1.0]
        ])
        # t7 = np.array([
        #     [1.0,       0.0,        0.0,        0.425],
        #     [0.0,       1.0,        0.0,        0.0],
        #     [0.0,       0.0,        1.0,        -0.0536635],
        #     [0.0,       0.0,        0.0,        1.0]
        # ])
        base_link_to_arm3_transformation = np.linalg.inv((t1 @ t2 @ t3 @ t4 @ t5 @ t6))
        setpoint_in_arm3_frame = base_link_to_arm3_transformation @ self._base_link_setpoint
        self._marker = Marker(
            header=Header(stamp=self.get_clock().now().to_msg(), frame_id="arm3"),
            ns="marker",
            id=0,
            type=Marker.ARROW,
            points=[Point(x=0.425, y=0.0, z=-0.0536635),
                    Point(x=setpoint_in_arm3_frame[0], y=setpoint_in_arm3_frame[1], z=setpoint_in_arm3_frame[2])],
            scale=Vector3(x=0.05, y=0.1, z=0.0),
            color=ColorRGBA(r=255.0, g=0.0, b=0.0, a=1.0)
            )
        self._marker_publisher.publish(self._marker)

def main():
    rclpy.init()
    rclpy.spin(RobotController())
    rclpy.shutdown()


if __name__ == '__main__':
    main()