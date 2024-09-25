import sys

from name_interface.srv import SummFullName
import rclpy
from rclpy.node import Node


class NameClient(Node):

    def __init__(self):
        super().__init__('name_client_async')
        self.cli = self.create_client(SummFullName, 'service_name')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SummFullName.Request()

    def send_request(self, first_name, last_name, surname):
        self.req.first_name = first_name
        self.req.last_name = last_name
        self.req.surname = surname
        return self.cli.call_async(self.req)


def main():
    rclpy.init()

    name_client = NameClient()
    future = name_client.send_request(sys.argv[1], sys.argv[2], sys.argv[3])
    rclpy.spin_until_future_complete(name_client, future)
    response = future.result()
    name_client.get_logger().info(
        f'Fullname is {response.full_name}')

    name_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()