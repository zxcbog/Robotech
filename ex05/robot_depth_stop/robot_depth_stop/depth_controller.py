from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from rclpy.node import Node
import rclpy
import numpy as np
import time

class Depth(Node):
     def __init__(self):
         super().__init__('depth')
         self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
         self.pose_subscriber = self.create_subscription(Image, '/depth/image', self.update_pose, 10)
         self.scan = Image()
         self.timer = self.create_timer(0.1, self.move)

     def update_pose(self, ranges):
         self.scan = ranges

     def move(self):
        vel_msg = Twist()
        window_size = 50
        
        if len(self.scan.data) != 0:
            depth_data = np.frombuffer(self.scan.data, dtype=np.float32).reshape(self.scan.height, self.scan.width)
            depth_data = np.nan_to_num(depth_data, nan=0.0, posinf=2.0, neginf=0.1)
            window = depth_data[self.scan.height-window_size//2:self.scan.height+window_size//2, 
                                     self.scan.width-window_size//2:self.scan.width+window_size//2]
            depth_value = window.sum() / (window_size ** 2)
            
            #self.get_logger().info(f'Current depth value at center: {depth_value}')
            
            if depth_value >= 0.1:
                vel_msg.linear.x = 0.3
            else:
                vel_msg.linear.x = 0.0

        vel_msg.angular.z = 0.0
        self.velocity_publisher.publish(vel_msg)
def main(args=None):
    rclpy.init(args=args)
    time.sleep(2)
    x = Depth()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()