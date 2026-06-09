import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleNode(Node):

    def __init__(self):
        super().__init__('turtle_node')
        self._logger = self.get_logger()
        self._subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self._publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        
        self._lin_vel = 5.0
        self._collided = False
        
    def pose_callback(self, pose: Pose):
        x, y = pose.x, pose.y
        vel = Twist()
        
        colliding = x < 2 or x > 9 or y < 2 or y > 9
        if colliding and not self._collided:
            self._lin_vel *= -1
            vel.angular.z = 40.0
            
        self._collided = colliding
        vel.linear.x = self._lin_vel
        self._publisher.publish(vel)

def main():
    rclpy.init()
    rclpy.spin(TurtleNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()