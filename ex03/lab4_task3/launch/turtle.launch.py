from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'delay',
            default_value='1.0',
            description='delay for turtle 2'
        ),
        Node(
            package='lab4_task3',
            executable='t1',
            name='t1_1',
            output='screen'
        ),
        Node(
            package='lab4_task3',
            executable='t2',
            name='t2_1',
            output='screen',
            parameters=[
                {'delay': LaunchConfiguration('delay')}
            ]
        ),
    ])

