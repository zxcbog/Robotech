tp start:
ros2 run turtle_move text_to_cmd_vel
using example:
ros2 topic pub --once /cmd_text std_msgs/msg/String "{data: 'turn_right 180'}"
