import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class SpeedMonitor(Node):
    def __init__(self):
        super().__init__('speed_monitor')

        self.declare_parameter('speed_limit', 50.0)
        self.declare_parameter('topic_name', 'vehicle_speed')

        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.speed_limit = self.get_parameter('speed_limit').get_parameter_value().double_value

        self.subscription = self.create_subscription(Float32, topic_name, self.listener_callback, 10)
        self.publisher_ = self.create_publisher(String, 'speed_feedback', 10)

    def listener_callback(self, msg):
        feedback = String()
        if msg.data > self.speed_limit:
            feedback.data = 'slow_down'
        elif msg.data < self.speed_limit:
            feedback.data = 'speed_up'
        else:
            feedback.data = 'maintain'
        self.publisher_.publish(feedback)
        self.get_logger().info(f'Received speed: {msg.data:.2f}, Feedback: {feedback.data}')

def main(args=None):
    rclpy.init(args=args)
    node = SpeedMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
