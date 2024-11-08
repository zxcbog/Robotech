import math

from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class DynamicFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('dynamic_frame_tf2_broadcaster')
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.01, self.broadcast_timer_callback)
        
        self.radius = self.declare_parameter(
          'radius', 3).get_parameter_value().integer_value
        
        self.direction_of_rotation = self.declare_parameter(
          'direction_of_rotation', 1).get_parameter_value().integer_value
        
        if not self.direction_of_rotation in [-1, 1]:
            self.get_logger().error(f'Bad direction_of_rotation: must be 1 (clockwise) or -1 (not clockwise)')
            return

    def broadcast_timer_callback(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        x = (seconds + nanoseconds // (1000 * 1000) / 1000) * math.pi / 10

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot1'
        t.transform.translation.x = self.direction_of_rotation * self.radius * math.cos(x)
        t.transform.translation.y = - self.radius * math.sin(x)
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = DynamicFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()