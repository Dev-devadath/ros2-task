import rclpy
from rclpy.node import Node
from msg_pkg.msg import Student

class StudentSubscriber(Node):
    def __init__(self):
        super().__init__('student_subscriber')
        self.subscriber_ = self.create_subscription(
            Student, 
            'student', 
            self.callback, 
            10
        )
        self.get_logger().info('Student Subscriber started. Waiting for student data...')

    def callback(self, msg):
        if msg.mark >= 50:
            status = "pass"
        else:
            status = "fail"
        
        msg.status = status
        
        self.get_logger().info(f'Student Name: {msg.name}')
        self.get_logger().info(f'Mark: {msg.mark}')
        self.get_logger().info(f'Status: {status.upper()}')

def main(args=None):
    rclpy.init(args=args)
    node = StudentSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

