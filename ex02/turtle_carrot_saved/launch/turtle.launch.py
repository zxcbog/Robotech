from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'radius',
            default_value='1.0',
            description='Radius of the carrot rotation around turtle1'
        ),
        DeclareLaunchArgument(
            'direction_of_rotation',
            default_value='1',
            description='Direction of rotation: 1 for clockwise, -1 for counterclockwise'
        ),
        Node(
            package='turtle_carrot',
            executable='turtle_carrot',
            name='turtle_carrot',
            output='screen',
            parameters=[
                {'radius': LaunchConfiguration('radius')},
                {'direction_of_rotation': LaunchConfiguration('direction_of_rotation')}
            ]
        ),
        Node(
            package='turtle_carrot',
            executable='turtle_follower',
            name='turtle_follower',
            output='screen'
        ),
    ])

