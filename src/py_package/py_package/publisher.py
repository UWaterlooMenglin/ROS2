import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher_node')
        self._logger = self.get_logger()
        self._publisher = self.create_publisher(String, 'topic', 10)
        self._timer = self.create_timer(1.0, self.publish_message)
        
        self._count = 0
        
    def publish_message(self):
        msg = String()
        msg.data = f"Hello ROS2!"
        self._publisher.publish(msg)
        self._count += 1
        self._logger.info(f"Published: {msg.data}, Count: {self._count}")

def main():
    rclpy.init()
    rclpy.spin(PublisherNode())
    rclpy.shutdown()

if __name__ == '__main__':
    main()