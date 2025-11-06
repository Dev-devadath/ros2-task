#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node

from pkg.srv import VowelCount


class VowelCountClient(Node):
    def __init__(self) -> None:
        super().__init__('vowel_count_client')
        self.cli = self.create_client(VowelCount, 'vowel_count')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, input_str: str) -> None:
        self.req = VowelCount.Request()
        self.req.input = input_str
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            response = self.future.result()
            self.get_logger().info(f'Request: "{input_str}"')
            self.get_logger().info(f'Response - Vowel count: {response.count}')
        else:
            self.get_logger().error(f'Service call failed {self.future.exception()}')


def parse_args(node: Node) -> str | None:
    if len(sys.argv) < 2:
        node.get_logger().error('Usage: vowel_count_client <string>')
        return None

    # Join all arguments after the script name to handle strings with spaces
    return ' '.join(sys.argv[1:])


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    vowel_count_client = VowelCountClient()

    parsed_args = parse_args(vowel_count_client)
    if parsed_args is None:
        vowel_count_client.destroy_node()
        rclpy.shutdown()
        return

    vowel_count_client.send_request(parsed_args)

    vowel_count_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

