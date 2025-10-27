import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String

class VehicleSimulator(Node):
    def __init__(self):
        super().__init__('vehicle_simulator')

        # Declare parameters
        self.declare_parameter('topic_name', 'vehicle_speed')
        self.declare_parameter('initial_rate', 1.0)  # Hz
        self.declare_parameter('speed_limit', 50.0)

        # Get parameters
        topic_name = self.get_parameter('topic_name').get_parameter_value().string_value
        self.speed_limit = self.get_parameter('speed_limit').get_parameter_value().double_value
        self.rate_hz = self.get_parameter('initial_rate').get_parameter_value().double_value

        # Publisher and subscriber
        self.publisher_ = self.create_publisher(Float32, topic_name, 10)
        self.subscription = self.create_subscription(String, 'speed_feedback', self.feedback_callback, 10)

        # Timer to publish speed
        self.speed = 30.0
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_speed)

    def publish_speed(self):
        msg = Float32()
        msg.data = self.speed
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing speed: {self.speed:.2f} km/h')

    def feedback_callback(self, msg):
        if msg.data == 'slow_down':
            self.rate_hz = max(0.5, self.rate_hz - 0.5)
        elif msg.data == 'speed_up':
            self.rate_hz += 0.5
        # Update timer period
        self.timer.cancel()
        self.timer = self.create_timer(1.0 / self.rate_hz, self.publish_speed)
        self.get_logger().info(f'Adjusted publishing rate: {self.rate_hz:.2f} Hz')

def main(args=None):
    rclpy.init(args=args)
    node = VehicleSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
