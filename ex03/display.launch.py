import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_name = "robot_with_sensors"
    pkg_share = launch_ros.substitutions.FindPackageShare(package=pkg_name).find(pkg_name)
    default_model_path = os.path.join(pkg_share, 'src/description/my_bot_description.urdf')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_topic = '/model/my_bot'
    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', LaunchConfiguration('model')])},
                    {'frame_prefix': "my_bot/"}]
    )
    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'robot_description': Command(['xacro ', default_model_path])}],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )
    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')],
    )
    
    gz_sim = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r gpu_lidar_sensor.sdf'}.items(),
    )
    spawn = launch_ros.actions.Node(
                package='ros_gz_sim', 
                executable='create',
                arguments=['-topic', '/robot_description'],
                output='screen')
    
    # Bridge
    ros_gz_bridge_node = launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{'config_file': os.path.join(pkg_share, 'config', 'robot_bridge.yaml')},
                   {'qos_overrides./tf_static.publisher.durability': 'transient_local'}],
        output='screen'
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(name='gui', default_value='True',
                                            description='Flag to enable joint_state_publisher_gui'),
        launch.actions.DeclareLaunchArgument(name='model', default_value=default_model_path,
                                            description='Absolute path to robot urdf file'),
        launch.actions.DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
                                            description='Absolute path to rviz config file'),
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
        gz_sim,
        spawn,
        ros_gz_bridge_node,
        # Node(
        #     package="tf2_ros",
        #     executable="static_transform_publisher",
        #     name='depth_frame_tf_publisher',
        #     arguments=['0', '0', '0.2', '-1.57', '0', '-1.57', '/my_bot/base_link', '/my_bot/depth_frame']
        # )
        
    ])