#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from pkg.srv import IntSign


class SignService(Node):
    def __init__(self) -> None:
        super().__init__('sign_service')
        self.srv = self.create_service(IntSign, 'sign_check', self.handle_request)

    def handle_request(self, request: IntSign.Request, response: IntSign.Response) -> IntSign.Response:
        value = request.value
        if value > 0:
            response.category = 'positive'
        elif value < 0:
            response.category = 'negative'
        else:
            response.category = 'zero'
        self.get_logger().info(f"value={value} -> {response.category}")
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


