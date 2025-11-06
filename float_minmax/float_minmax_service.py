#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from pkg.srv import FloatMinMax


class FloatMinMaxService(Node):
    def __init__(self) -> None:
        super().__init__('float_minmax_service')
        self.srv = self.create_service(FloatMinMax, 'float_minmax', self.handle_request)

    def handle_request(self, request: FloatMinMax.Request, response: FloatMinMax.Response) -> FloatMinMax.Response:
        numbers = request.numbers
        if len(numbers) == 0:
            self.get_logger().warn("Received empty array")
            response.max = 0.0
            response.min = 0.0
            return response
        
        response.max = max(numbers)
        response.min = min(numbers)
        self.get_logger().info(f"Received {len(numbers)} numbers: {numbers}")
        self.get_logger().info(f"Max: {response.max}, Min: {response.min}")
        return response


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = FloatMinMaxService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

