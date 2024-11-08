import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    radius_arg = DeclareLaunchArgument(
        'radius', default_value='1', description='Radius of the rotation'
    )
    direction_of_rotation_arg = DeclareLaunchArgument(
        'direction_of_rotation', default_value='1', description='Direction of rotation (1 for clockwise, -1 for counterclockwise)'
    )
    demo_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('turtle_carrot'), 'launch'),
            '/turtle.launch.py']),
       launch_arguments={'target_frame': 'carrot1'}.items(),
       )

    return LaunchDescription([
        radius_arg,
        direction_of_rotation_arg,
        demo_nodes,
        Node(
            package='turtle_carrot',
            executable='dynamic_frame_broadcaster',
            name='dynamic_broadcaster',
            parameters=[{
                'radius': LaunchConfiguration('radius'),
                'direction_of_rotation': LaunchConfiguration('direction_of_rotation')
            }]
        ),
        
    ])