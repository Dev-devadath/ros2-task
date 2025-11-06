#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node

from pkg.srv import IntSign


class SignClient(Node):
    def __init__(self) -> None:
        super().__init__('sign_client')
        self.cli = self.create_client(IntSign, 'sign_check')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, numbers: list[int]) -> None:
        self.req = IntSign.Request()
        self.req.numbers = numbers
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            response = self.future.result()
            self.get_logger().info(f'Request: {numbers}')
            self.get_logger().info(f'Response (even numbers): {response.even_numbers}')
        else:
            self.get_logger().error(f'Service call failed {self.future.exception()}')


def parse_args(node: Node) -> list[int] | None:
    if len(sys.argv) < 2:
        node.get_logger().error('Usage: sign_client <number1> <number2> ... <numberN>')
        return None

    try:
        return [int(arg) for arg in sys.argv[1:]]
    except ValueError:
        node.get_logger().error('All arguments must be integers')
        return None


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    sign_client = SignClient()

    parsed_args = parse_args(sign_client)
    if parsed_args is None:
        sign_client.destroy_node()
        rclpy.shutdown()
        return

    sign_client.send_request(parsed_args)

    sign_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

