import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

import math

class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')
        self._logger = self.get_logger()
        self._publisher = self.create_publisher(JointState, "/joint_states", 5)
        self.create_timer(0.02, self.robot_controller_callback)
        
    def robot_controller_callback(self):
        jointState = JointState()
        now = self.get_clock().now()
        now_sec = now.nanoseconds / 1e9
        position = (math.pi / 2) * math.sin(((2 * math.pi) / 5) * now_sec)
        
        jointState.header.stamp = now.to_msg()
        jointState.name = ["arm1_base", "arm1", "arm2_base", "arm2"]
        jointState.position = [position, position, position, position]
        
        self._publisher.publish(jointState)

def main():
    rclpy.init()
    rclpy.spin(RobotController())
    rclpy.shutdown()

if __name__ == '__main__':
    main()