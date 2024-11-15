from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
import rclpy
import time

class Lidar(Node):
     def __init__(self):
         super().__init__('lidar')
         self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
         self.pose_subscriber = self.create_subscription(LaserScan, '/my_bot/scan', self.update_pose, 10)
         self.scan = LaserScan()
         self.timer = self.create_timer(0.5, self.move)

     def update_pose(self, ranges):
         self.scan = ranges

     def move(self):
         vel_msg = Twist()
         laser = self.scan.ranges
         vel_msg.linear.x = 0.
         if (len(laser)!=0):
            if (laser[179]<0.41):
                vel_msg.linear.x = 0.
            else:
                vel_msg.linear.x = 0.3
         vel_msg.angular.z = 0.0
         self.velocity_publisher.publish(vel_msg)
def main(args=None):
    rclpy.init(args=args)
    time.sleep(2)
    x = Lidar()
    rclpy.spin(x)
    x.destroy_node()
    rclpy.shutdown()

 
if __name__ == '__main__':
    main()