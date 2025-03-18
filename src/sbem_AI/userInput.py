import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class UserInputPublisher(Node):
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher_ = self.create_publisher(String, '/user_input', 10)
        self.subscription = self.create_subscription(String, '/response_to_user', self.listener_callback, 1)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def listener_callback(self, msg):
        self.get_logger().info(f'SBEM ha risposto : "{msg.data}"')
        
    def timer_callback(self):
        msg = String()
        msg.data = input("Enter your message: ")
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    user_input_publisher = UserInputPublisher()
    rclpy.spin(user_input_publisher)
    user_input_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()