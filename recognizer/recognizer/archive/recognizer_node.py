import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Recognizer(Node):
    def __init__(self):
        print('Initializing Recognizer...')
        super().__init__('Recognizer_node')
        #self.get_logger().info('HappyNode has been started!')
        self.pub = self.create_publisher(String, 'happy_topic', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.i = 10

    def timer_callback(self):
        msg = String()
        msg.data = f'Happy message {self.i}'
        self.pub.publish(msg)
        self.i += 1
        self.pub.publish(msg)
        self.get_logger().info('HappyNode is running happily!')

def main():
    print('Hi from recognizer.')
    rclpy.init()
    node = Recognizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('KeyboardInterrupt received, shutting down HappyNode...')
    node.destroy_node()
    rclpy.try_shutdown()
    print('HappyNode has been shut down.')

if __name__ == '__main__':
    main()
