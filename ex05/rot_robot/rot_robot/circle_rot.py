import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class RobotRotator(Node):

    def __init__(self):
        super().__init__('circle_rot')

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )
        self.timer = self.create_timer(0.1, self.rotate)
        self.counter = 0
        self.limit = 30
        
        
    def rotate(self):
        command = Twist()
        if self.counter < self.limit:
            command.linear.x = 0.5
            command.angular.z = 0.5
        else:
            command.linear.x = 0.5
            command.angular.z = -0.5
        if self.counter > self.limit * 2:
            self.counter = 0
        self.counter += 1
        self.publisher.publish(command)
        

def main(args=None):
    rclpy.init(args=args)

    rot = RobotRotator()
    
    

    rclpy.spin(rot)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    rot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
