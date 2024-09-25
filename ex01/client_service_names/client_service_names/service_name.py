from name_interface.srv import SummFullName

import rclpy
from rclpy.node import Node


class NameService(Node):

    def __init__(self):
        super().__init__('service_name')
        self.srv = self.create_service(SummFullName, 'service_name', self.make_full_name)

    def make_full_name(self, request, response):
        self.get_logger().info('New request has received')
        response.full_name = f"{request.first_name} {request.last_name} {request.surname}"
        self.get_logger().info('Request has proccessed succesfully')

        return response


def main():
    rclpy.init()

    minimal_service = NameService()

    rclpy.spin(minimal_service)

    rclpy.shutdown()


if __name__ == '__main__':
    main()