import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from turtle_commands.action import MessageTurtleCommands
from std_msgs.msg import String

class TurtleActionClient(Node):

    def __init__(self):
        super().__init__('action_turtle_client')
        
        self.get_logger().info(f'Starting up the client...')
        self._action_client = ActionClient(self, MessageTurtleCommands, 'action_turtle_server')
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.decode_message,
            10
        )

    def send_goal(self, command, distance=0, angle=0):
        goal_msg = MessageTurtleCommands.Goal()
        goal_msg.command = command
        goal_msg.s = distance
        goal_msg.angle = angle

        self._action_client.wait_for_server()
        
        self.get_logger().info(f'Goal with data {command} {distance if distance else angle}({"distance" if distance else "angle"}) has sent')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            return

        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result.result}')

    def decode_message(self, msg):
        print(f"Received message: {msg}")

        # msg must be like : "turn_right value"

        data = msg.data.split(" ")
        if len(data) != 2 or data[0] not in ["turn_right", "turn_left", "forward"]:
            print("Unknown command...")
            return
        distance = 0
        angle = 0
        if data[0] == "turn_right" or data[0] == "turn_left":
            angle = int(data[1])
        else:
            distance = int(data[1])
        self.send_goal(data[0], angle=angle,distance=distance)

def main(args=None):
    rclpy.init(args=args)
    action_client = TurtleActionClient()

    #action_client.send_goal('forward', distance=2)
    rclpy.spin(action_client)
    rclpy.shutdown()

if __name__ == '__main__':
    main()