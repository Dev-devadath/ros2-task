#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from pkg.srv import IntSign


class SignService(Node):
    def __init__(self) -> None:
        super().__init__('sign_service')
        self.srv = self.create_service(IntSign, 'sign_check', self.handle_request)

    def handle_request(self, request: IntSign.Request, response: IntSign.Response) -> IntSign.Response:
        numbers = request.numbers
        # Filter even numbers (numbers divisible by 2)
        even_numbers = [num for num in numbers if num % 2 == 0]
        response.even_numbers = even_numbers
        self.get_logger().info(f"Received {len(numbers)} numbers: {numbers}")
        self.get_logger().info(f"Returning {len(even_numbers)} even numbers: {even_numbers}")
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = SignService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


