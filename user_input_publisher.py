#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class UserInputPublisher(Node):
    
    def __init__(self):
        super().__init__('user_input_publisher')
        self.publisher_ = self.create_publisher(String, '/user_input', 10)
        
        self.subscription = self.create_subscription(
            String,
            '/chatbot_response',
            self.chatbot_response_callback,
            10
        )
        
        self.get_logger().info('User Input Publisher started!')
        self.get_logger().info('Type your messages and press Enter.')
        self.get_logger().info('Press Ctrl+C to exit.\n')
    
    def chatbot_response_callback(self, msg):
        print(f"\n🤖 Chatbot: {msg.data}\n")
        print("You: ", end='', flush=True)
    
    def publish_input(self):
        print("You: ", end='', flush=True)
        try:
            user_message = input()
            if user_message:
                msg = String()
                msg.data = user_message
                self.publisher_.publish(msg)
        except EOFError:
            return False
        return True


def main(args=None):
    """
    Main function to run the user input publisher.
    """
    rclpy.init(args=args)
    
    node = UserInputPublisher()
    
    try:
        while rclpy.ok():
            # Process any pending callbacks
            rclpy.spin_once(node, timeout_sec=0.1)
            
            # Get user input (non-blocking with timeout)
            import select
            import sys
            
            # Check if there's input available
            if select.select([sys.stdin], [], [], 0.1)[0]:
                if not node.publish_input():
                    break
                    
    except KeyboardInterrupt:
        node.get_logger().info('\nShutting down user input publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

