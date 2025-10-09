#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import threading


class StudentNode(Node):
    def __init__(self):
        super().__init__('student_node')
        
        # Publisher
        self.answer_publisher = self.create_publisher(String, 'quiz_answer', 10)
        
        # Subscribers
        self.question_subscriber = self.create_subscription(
            String,
            'quiz_question',
            self.question_callback,
            10
        )
        
        self.feedback_subscriber = self.create_subscription(
            String,
            'quiz_feedback',
            self.feedback_callback,
            10
        )
        
        self.current_question = None
        self.waiting_for_input = False
        self.quiz_ended = False
        
        self.get_logger().info('Student Node Started!')
        self.get_logger().info('Waiting for quiz questions from Quiz Master...')
        self.get_logger().info('=' * 60)
    
    def question_callback(self, msg):
        """Receive question from quiz master"""
        if self.quiz_ended:
            return
        
        self.current_question = msg.data
        self.waiting_for_input = True
        
        # Display question
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('NEW QUESTION:')
        self.get_logger().info(msg.data)
        self.get_logger().info('=' * 60)
        
        # Start input thread
        input_thread = threading.Thread(target=self.get_user_input)
        input_thread.daemon = True
        input_thread.start()
    
    def get_user_input(self):
        """Get answer from user via terminal input"""
        if self.quiz_ended:
            return
        
        try:
            print('\nYour answer: ', end='', flush=True)
            answer = input().strip()
            
            if answer and self.waiting_for_input and not self.quiz_ended:
                # Publish answer
                msg = String()
                msg.data = answer
                self.answer_publisher.publish(msg)
                self.get_logger().info(f'Answer submitted: {answer}')
                self.waiting_for_input = False
            elif not answer:
                self.get_logger().warn('Empty answer provided. Please try again.')
                self.get_user_input()  # Retry
        except EOFError:
            self.get_logger().warn('Input stream closed')
        except Exception as e:
            self.get_logger().error(f'Error getting input: {e}')
    
    def feedback_callback(self, msg):
        """Receive feedback from quiz master"""
        feedback = msg.data
        
        # Check if quiz has ended
        if feedback.startswith('QUIZ_END'):
            self.quiz_ended = True
            parts = feedback.split('|')
            if len(parts) == 4:
                score = parts[1]
                total = parts[2]
                percentage = parts[3]
                
                self.get_logger().info('\n' + '=' * 60)
                self.get_logger().info('QUIZ COMPLETED!')
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'YOUR FINAL SCORE: {score}/{total} ({percentage}%)')
                self.get_logger().info('=' * 60)
                self.get_logger().info('Thank you for participating!')
                self.get_logger().info('Press Ctrl+C to exit.')
        else:
            # Regular feedback
            self.get_logger().info('\n' + '-' * 60)
            self.get_logger().info(f'FEEDBACK: {feedback}')
            self.get_logger().info('-' * 60)
            self.get_logger().info('Waiting for next question...\n')


def main(args=None):
    rclpy.init(args=args)
    student_node = StudentNode()
    
    try:
        rclpy.spin(student_node)
    except KeyboardInterrupt:
        student_node.get_logger().info('Student Node shutting down...')
    finally:
        student_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

