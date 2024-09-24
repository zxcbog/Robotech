import rclpy
from rclpy.node import Node

from std_msgs.msg import String

from geometry_msgs.msg import Twist

class TurtleMovement(Node):

    def __init__(self):
        super().__init__('turtle_movement')
        
        self.subscription = self.create_subscription(
            String,
            'cmd_text',
            self.decode_message,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

    def decode_message(self, msg):
        print(f"Received message: {msg}")

        # msg must be like : "turn_right value"

        data = msg.data.split(" ")
        if len(data) != 2 or data[0] not in ["turn_right", "turn_left", "move_forward", "move_backward"]:
            print("Unknown command...")
            return
        value = 1
        try:
            value = float(data[1])
        except Exception as e:
            print(f"[ERROR]: {e}")
            return
        
        response = Twist()

        if data[0] == "turn_left":
            response.angular.z = value * 3.14 / 180
        elif data[0] == "turn_right":
            response.angular.z = -1 * value * 3.14 / 180
        elif data[0] == "move_forward":
            response.linear.x = value
        elif data[0] == "move_backward":
            response.linear.x = -value

        self.publisher.publish(response)
        print("Command was succesfully executed")


def main(args=None):
    rclpy.init(args=args)

    turtle = TurtleMovement()

    rclpy.spin(turtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
