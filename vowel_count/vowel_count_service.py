#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from pkg.srv import VowelCount


class VowelCountService(Node):
    def __init__(self) -> None:
        super().__init__('vowel_count_service')
        self.srv = self.create_service(VowelCount, 'vowel_count', self.handle_request)

    def handle_request(self, request: VowelCount.Request, response: VowelCount.Response) -> VowelCount.Response:
        input_str = request.input
        vowels = 'aeiouAEIOU'
        count = sum(1 for char in input_str if char in vowels)
        response.count = count
        self.get_logger().info(f"Received string: '{input_str}'")
        self.get_logger().info(f"Vowel count: {count}")
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = VowelCountService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

