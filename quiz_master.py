#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class QuizMaster(Node):
    def __init__(self):
        super().__init__('quiz_master')
        
        # Publishers
        self.question_publisher = self.create_publisher(String, 'quiz_question', 10)
        self.feedback_publisher = self.create_publisher(String, 'quiz_feedback', 10)
        
        # Subscriber
        self.answer_subscriber = self.create_subscription(
            String,
            'quiz_answer',
            self.answer_callback,
            10
        )
        
        # Quiz data
        self.questions = [
            {
                'question': 'Q1: What is 15 + 27? ',
                'answer': '42',
                'type': 'math'
            },
            {
                'question': 'Q2: True or False: Python is a compiled language? ',
                'answer': 'False',
                'type': 'true_false'
            },
            {
                'question': 'Q3: What is 8 * 9? ',
                'answer': '72',
                'type': 'math'
            },
            {
                'question': 'Q4: Multiple Choice: What does ROS stand for?\nA) Robot Operating System\nB) Remote Operation Software\nC) Robotic Output Service\nD) Real-time OS\n',
                'answer': 'A',
                'type': 'multiple_choice'
            },
            {
                'question': 'Q5: True or False: I know im dumb! ',
                'answer': 'True',
                'type': 'true_false'
            }
        ]
        
        self.current_question = 0
        self.score = 0
        self.total_questions = len(self.questions)
        self.waiting_for_answer = False
        self.quiz_active = True
        
        self.get_logger().info('Quiz Master Node Started!')
        self.get_logger().info('=' * 60)
        self.get_logger().info('WELCOME TO THE ROS 2 QUIZ GAME!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Total Questions: {self.total_questions}')
        self.get_logger().info('Starting quiz in 2 seconds...')
        self.get_logger().info('Make sure student_node is running to participate!')
        
        # Timer to start publishing questions
        self.timer = self.create_timer(2.0, self.start_quiz)
    
    def start_quiz(self):
        """Start the quiz by publishing the first question"""
        self.timer.cancel()  # Cancel the start timer
        self.publish_question()
    
    def publish_question(self):
        """Publish the current question"""
        if self.current_question < self.total_questions and self.quiz_active:
            question_data = self.questions[self.current_question]
            msg = String()
            msg.data = question_data['question']
            
            self.get_logger().info('\n' + '=' * 60)
            self.get_logger().info(f'Publishing Question {self.current_question + 1}/{self.total_questions}:')
            self.get_logger().info(question_data['question'])
            self.get_logger().info('=' * 60)
            
            self.question_publisher.publish(msg)
            self.waiting_for_answer = True
        elif self.quiz_active:
            # Quiz completed
            self.quiz_active = False
            self.publish_final_score()
    
    def answer_callback(self, msg):
        """Process received answers from student"""
        if not self.waiting_for_answer or not self.quiz_active:
            return
        
        student_answer = msg.data.strip()
        correct_answer = self.questions[self.current_question]['answer']
        
        self.get_logger().info(f'\nReceived answer: {student_answer}')
        
        # Check if answer is correct (case-insensitive)
        is_correct = student_answer.lower() == correct_answer.lower()
        
        if is_correct:
            self.score += 1
            feedback = f'✓ CORRECT! The answer is {correct_answer}. Score: {self.score}/{self.current_question + 1}'
            self.get_logger().info(f'✓ CORRECT! Current Score: {self.score}/{self.current_question + 1}')
        else:
            feedback = f'✗ INCORRECT. The correct answer is {correct_answer}. Score: {self.score}/{self.current_question + 1}'
            self.get_logger().info(f'✗ INCORRECT. Correct answer was: {correct_answer}. Current Score: {self.score}/{self.current_question + 1}')
        
        # Publish feedback
        feedback_msg = String()
        feedback_msg.data = feedback
        self.feedback_publisher.publish(feedback_msg)
        
        # Move to next question
        self.waiting_for_answer = False
        self.current_question += 1
        
        # Wait a bit before next question
        time.sleep(2)
        self.publish_question()
    
    def publish_final_score(self):
        """Publish final score and end quiz"""
        percentage = (self.score / self.total_questions) * 100
        
        self.get_logger().info('\n' + '=' * 60)
        self.get_logger().info('QUIZ COMPLETED!')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'FINAL SCORE: {self.score}/{self.total_questions} ({percentage:.1f}%)')
        
        if percentage == 100:
            self.get_logger().info('🏆 PERFECT SCORE! Excellent work!')
        elif percentage >= 80:
            self.get_logger().info('⭐ Great job! You did very well!')
        elif percentage >= 60:
            self.get_logger().info('👍 Good effort! Keep practicing!')
        else:
            self.get_logger().info('📚 Keep learning! You\'ll do better next time!')
        
        self.get_logger().info('=' * 60)
        
        # Publish final feedback
        final_msg = String()
        final_msg.data = f'QUIZ_END|{self.score}|{self.total_questions}|{percentage:.1f}'
        self.feedback_publisher.publish(final_msg)


def main(args=None):
    rclpy.init(args=args)
    quiz_master = QuizMaster()
    
    try:
        rclpy.spin(quiz_master)
    except KeyboardInterrupt:
        quiz_master.get_logger().info('Quiz Master shutting down...')
    finally:
        quiz_master.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

