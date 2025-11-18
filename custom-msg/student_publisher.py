import rclpy
from rclpy.node import Node
from msg_pkg.msg import Student

class StudentPublisher(Node):
    def __init__(self):
        super().__init__('student_publisher')
        self.publisher_ = self.create_publisher(Student, 'student', 10)
        self.timer_ = self.create_timer(2.0, self.publish_student)
        self.student_count = 0
        
        self.students = [
            {'name': 'Alice', 'mark': 85},
            {'name': 'Bob', 'mark': 45},
            {'name': 'Charlie', 'mark': 92},
            {'name': 'Diana', 'mark': 38},
            {'name': 'Eve', 'mark': 67},
        ]

    def publish_student(self):
        if self.student_count < len(self.students):
            student_data = self.students[self.student_count]
            student = Student()
            student.name = student_data['name']
            student.mark = student_data['mark']
            student.status = ""  
            
            self.publisher_.publish(student)
            self.get_logger().info(f'Published student: {student.name}, mark: {student.mark}')
            self.student_count += 1
        else:
            self.get_logger().info('All students published. Stopping...')
            self.timer_.cancel()

def main(args=None):
    rclpy.init(args=args)
    node = StudentPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

