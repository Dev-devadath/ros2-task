#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from pkg.srv import StringEqual


class StringEqualService(Node):
    def __init__(self) -> None:
        super().__init__('string_equal_service')
        self.srv = self.create_service(StringEqual, 'string_equal', self.handle_request)

    def handle_request(self, request: StringEqual.Request, response: StringEqual.Response) -> StringEqual.Response:
        str1 = request.str1
        str2 = request.str2
        response.are_equal = (str1 == str2)
        self.get_logger().info(f"Received str1: '{str1}', str2: '{str2}'")
        self.get_logger().info(f"Are equal: {response.are_equal}")
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = StringEqualService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

