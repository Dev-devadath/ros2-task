#!/usr/bin/env python3
import sys

import rclpy
from rclpy.node import Node

from pkg.srv import StringEqual


class StringEqualClient(Node):
    def __init__(self) -> None:
        super().__init__('string_equal_client')
        self.cli = self.create_client(StringEqual, 'string_equal')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

    def send_request(self, str1: str, str2: str) -> None:
        self.req = StringEqual.Request()
        self.req.str1 = str1
        self.req.str2 = str2
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        if self.future.result() is not None:
            response = self.future.result()
            self.get_logger().info(f'Request - str1: "{str1}", str2: "{str2}"')
            self.get_logger().info(f'Response - Are equal: {response.are_equal}')
        else:
            self.get_logger().error(f'Service call failed {self.future.exception()}')


def parse_args(node: Node) -> tuple[str, str] | None:
    if len(sys.argv) != 3:
        node.get_logger().error('Usage: string_equal_client <str1> <str2>')
        return None

    return sys.argv[1], sys.argv[2]


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    string_equal_client = StringEqualClient()

    parsed_args = parse_args(string_equal_client)
    if parsed_args is None:
        string_equal_client.destroy_node()
        rclpy.shutdown()
        return

    str1, str2 = parsed_args
    string_equal_client.send_request(str1, str2)

    string_equal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

