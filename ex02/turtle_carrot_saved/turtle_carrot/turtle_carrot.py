import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, Quaternion, Twist
from turtlesim.msg import Pose
import tf2_ros
from tf2_ros import Buffer
import math

class TurtleTf2Broadcaster(Node):
    def __init__(self):
        super().__init__('turtle_tf2_broadcaster')
        self.declare_parameter('radius', 1.0)
        self.declare_parameter('direction_of_rotation', 1)

        self.radius = self.get_parameter('radius').value
        self.direction_of_rotation = self.get_parameter('direction_of_rotation').value
        self.current_turtle_pos = None
        self.tf_buffer = Buffer()
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.get_pose_info,
            10
        )
        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.broadcast_transform)
        self.angle = 0.0
        
    def get_pose_info(self, msg):
        self.current_turtle_pos = msg

    def broadcast_transform(self):
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle1'
        t.child_frame_id = 'carrot'
        movement = Twist()
        
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('turtle2', 'turtle', now)
        except Exception as e:
            self.get_logger().error(f'Could not transform: {str(e)}')
            trans = TransformStamped()
            trans.transform.translation.x = 2.0
            trans.transform.translation.y = 2.0
        new_x = trans.transform.translation.x + self.radius * math.cos(self.angle)
        new_y = trans.transform.translation.y + self.radius * math.sin(self.angle)
        if self.current_turtle_pos:
            x_diff = new_x - self.current_turtle_pos.x
            y_diff = new_y - self.current_turtle_pos.y
            theta = self.current_turtle_pos.theta
        else:
            x_diff = 0
            y_diff = 0
            theta = 0
        
        angle_to_rotate = (math.atan2(y_diff, x_diff) - theta + math.pi) % (2 * math.pi) - math.pi
        self.get_logger().info(f"angle_to_rotate: {self.angle}")
        movement.linear.x = 2.0
        movement.angular.z = angle_to_rotate
        self.publisher.publish(movement)
        
        t.transform.translation.x = new_x
        t.transform.translation.y = new_y
        t.transform.translation.z = 0.0
        #self.get_logger().info(f'X pos: {t.transform.translation.x} Y pos: {t.transform.translation.y}')
        #self.get_logger().info(f'X pos: {self.current_turtle_pos.x} Y pos: {self.current_turtle_pos.y}')

        self.tf_broadcaster.sendTransform(t)
        self.get_logger().info(f"{math.sqrt(x_diff * x_diff + y_diff * y_diff)}")
        # Увеличиваем угол для движения фрейма
        if math.sqrt(x_diff * x_diff + y_diff * y_diff) < 1.5:
            self.angle = (self.direction_of_rotation * math.pi/40 + self.angle) % (math.pi * 2)

    def euler_to_quaternion(self, roll, pitch, yaw):
        q = Quaternion()

        # Формулы для преобразования углов Эйлера в кватернионы
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q.w = cr * cp * cy + sr * sp * sy
        q.x = sr * cp * cy - cr * sp * sy
        q.y = cr * sp * cy + sr * cp * sy
        q.z = cr * cp * sy - sr * sp * cy

        return q

def main(args=None):
    rclpy.init(args=args)
    node = TurtleTf2Broadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
