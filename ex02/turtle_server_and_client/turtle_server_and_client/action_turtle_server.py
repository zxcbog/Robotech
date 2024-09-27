import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
import math
from turtle_commands.action import MessageTurtleCommands

from turtlesim.msg import Pose
import time
from geometry_msgs.msg import Twist

class TurtleServer(Node):

    def __init__(self):
        super().__init__('action_turtle_server')
        self.get_logger().info(f'Starting up the server...')
        self._action_server = ActionServer(
            self,
            MessageTurtleCommands,
            'action_turtle_server',
            self.execute_callback)
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
        self.angle_commands = {
            'turn_left': 1,
            'turn_right': -1
        }
        self.current_turtle_pos = None
        self.get_logger().info(f'Server is ready to work')

    def get_pose_info(self, msg):
        #self.get_logger().info(f'current pose is: {msg}')
        self.current_turtle_pos = msg

    def execute_callback(self, goal_handle):
        self.get_logger().info('Goal has received!')
        command = goal_handle.request.command
        result = MessageTurtleCommands.Result()
        feedback_msg = MessageTurtleCommands.Feedback()
        movement = Twist()
        self.create_rate(10)
        start_pose = self.current_turtle_pos
        if start_pose is None:
            result.result = False
            goal_handle.succeed()
            self.get_logger().error("TurtleSim is not running")
            return result
        try:
            if goal_handle.is_cancel_requested:
                result.result = False
                goal_handle.succeed()
                self.get_logger().error("Goal was canceled at runtime")
                return result
            if command in ['forward']:
                movement.linear.x = float(goal_handle.request.s)
            elif command in ['turn_left', 'turn_right']:
                movement.angular.z = self.angle_commands[command] * goal_handle.request.angle * 3.14 / 180
            self.publisher.publish(movement)
            distance_moved = int(math.sqrt(
                (self.current_turtle_pos.x - start_pose.x)**2 +
                (self.current_turtle_pos.y - start_pose.y)**2
            ))
            feedback_msg.odom = distance_moved
            self.get_logger().info(f'Turtle has moved on {feedback_msg.odom} m')
            goal_handle.publish_feedback(feedback_msg)
            result.result = True
        except Exception as e:
            self.get_logger().error("Unexpected goal arguments")
            self.get_logger().error(f"{e}")
            goal_handle.succeed()
            result.result = False
            return result
        
        goal_handle.succeed()
        
        return result


def main(args=None):
    rclpy.init(args=args)
    turtle_server = TurtleServer()

    rclpy.spin(turtle_server)


if __name__ == '__main__':
    main()