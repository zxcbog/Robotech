import rclpy
from rclpy.node import Node
import sys
import time
import math

from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class TurtleTeleport(Node):

    def __init__(self, x, y):
        super().__init__('turtle_movement')
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.get_pose_info,
            10
        )
        
        self.x = x
        self.y = y

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )
        self.current_turtle_pos = None  
        self.should_teleport = True
        
        
    def teleport_turtle(self):
        print(self.current_turtle_pos)
        x_diff = self.x - self.current_turtle_pos.x
        y_diff = self.y - self.current_turtle_pos.y
        distance = math.sqrt(x_diff * x_diff + y_diff * y_diff)
        theta = self.current_turtle_pos.theta
        print(math.atan2(y_diff, x_diff))
        angle_to_rotate = math.atan2(y_diff, x_diff) - theta
        
        new_rot = Twist()
        new_rot.angular.z = angle_to_rotate
        
        self.publisher.publish(new_rot)
        
        time.sleep(1)
        
        new_pos = Twist()
        print(distance)
        new_pos.linear.x = distance
        self.publisher.publish(new_pos)
        

    def get_pose_info(self, msg):
        self.current_turtle_pos = msg
        if self.should_teleport:
            self.teleport_turtle()
            self.should_teleport = False


def main(args=None):
    rclpy.init(args=args)

    target_x = float(sys.argv[1])
    target_y = float(sys.argv[2])

    turtle = TurtleTeleport(target_x, target_y)
    
    

    rclpy.spin(turtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
