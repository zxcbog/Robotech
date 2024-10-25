import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, TransformStamped, Quaternion
from tf2_ros import TransformListener, Buffer
from math import atan2, sqrt, pi
import tf2_ros

class Turtle2Follower(Node):
    def __init__(self):
        super().__init__('turtle2_follower')

        # Создаем клиента для спавна черепахи
        self.spawn_client = self.create_client(Spawn, 'spawn')
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        self.spawn_turtle2()

        # Создаем подписку на tf2 и подписываемся на трансформации
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.get_pose_info,
            10
        )
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.publisher = self.create_publisher(Twist, '/turtle2/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.follow_carrot)
    
    def get_pose_info(self, msg):
        self.current_turtle_pos = msg

    def spawn_turtle2(self):
        # Спавним вторую черепаху
        spawn_req = Spawn.Request()
        spawn_req.x = 4.0
        spawn_req.y = 2.0
        spawn_req.theta = 0.0
        spawn_req.name = 'turtle2'

        future = self.spawn_client.call_async(spawn_req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Spawned turtle2 successfully.')
        else:
            self.get_logger().error('Failed to spawn turtle2')

    def follow_carrot(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'turtle2'
        t.child_frame_id = 'turtle'
        current_coord = [4, 2]
        theta = 0
        if self.current_turtle_pos:
            current_coord[0] = self.current_turtle_pos.x
            current_coord[1] = self.current_turtle_pos.y
            theta = self.current_turtle_pos.theta
        t.transform.translation.x = current_coord[0]
        t.transform.translation.y = current_coord[1]
        t.transform.translation.z = 0.0
        self.tf_broadcaster.sendTransform(t)
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('turtle1', 'carrot', now)
            # Расчёт дистанции и угла до "морковки"
            x_diff = trans.transform.translation.x - current_coord[0]
            y_diff = trans.transform.translation.y - current_coord[1]
            distance = sqrt(x_diff ** 2 + y_diff ** 2)
            angle_to_goal = (atan2(y_diff, x_diff) - theta + pi) % (2 * pi) - pi
            
            # Публикация скоростей для движения черепахи к морковке
            cmd_vel = Twist()
            if distance > 0.5:
                cmd_vel.linear.x = 1.0
                cmd_vel.angular.z = angle_to_goal
            else:
                cmd_vel.linear.x = 0.0
                cmd_vel.angular.z = 0.0

            self.publisher.publish(cmd_vel)

        except Exception as e:
            self.get_logger().error(f'Could not transform: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = Turtle2Follower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
