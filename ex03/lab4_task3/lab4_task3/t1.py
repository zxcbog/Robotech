from geometry_msgs.msg import TransformStamped

from turtlesim.msg import Pose
import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster


class FixedFrameBroadcaster(Node):

    def __init__(self):
        super().__init__('fixed_frame_tf2_broadcaster')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.get_pose_info,
            10
        )
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
        self.current_turtle_pos = None
    
    def get_pose_info(self, msg):
        self.current_turtle_pos = msg

    def broadcast_timer_callback(self):
        t = TransformStamped()
        if not self.current_turtle_pos:
            return
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot'
        t.transform.translation.x = self.current_turtle_pos.x
        t.transform.translation.y = self.current_turtle_pos.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = FixedFrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()