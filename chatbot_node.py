#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random


class ChatbotNode(Node):
    
    def __init__(self):
        super().__init__('chatbot_node')
        
        self.publisher_ = self.create_publisher(String, '/chatbot_response', 10)
        
        self.subscription = self.create_subscription(
            String,
            '/user_input',
            self.user_input_callback,
            10
        )
        
        self.message_counter = 0
        
        self.encouragement_list = [
            "Keep it up!",
            "Good!",
            "Yeee!",
        ]
        
        self.get_logger().info('Chatbot node initialized and ready!')
        self.get_logger().info('Waiting for messages on /user_input...')
    
    def user_input_callback(self, msg):
        """
        Callback function that processes user input and generates responses.
        """
        self.message_counter += 1
        
        user_text = msg.data.strip().lower()
        
        self.get_logger().info(f'Received message #{self.message_counter}: "{msg.data}"')
        
        response = String()
        
        if "hello" in user_text or "hi" in user_text:
            response.data = "Hi! How are you?"
        elif "bye" in user_text or "goodbye" in user_text:
            response.data = "Goodbye!"
        elif "help" in user_text:
            response.data = random.choice(self.encouragement_list)
        else:
            response.data = f"I heard you say: '{msg.data}'. Try saying 'hello', 'bye', or 'help'!"
        
        self.publisher_.publish(response)
        self.get_logger().info(f'Response sent: "{response.data}"')
        self.get_logger().info(f'Total messages received: {self.message_counter}')


def main(args=None):
    """
    Main function to initialize and run the chatbot node.
    """
    # Initialize ROS 2
    rclpy.init(args=args)
    
    chatbot_node = ChatbotNode()
    
    try:
        rclpy.spin(chatbot_node)
    except KeyboardInterrupt:
        chatbot_node.get_logger().info('Chatbot shutting down...')
    finally:
        chatbot_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

