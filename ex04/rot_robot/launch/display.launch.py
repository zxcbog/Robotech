import launch
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import launch_ros
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_name = "rot_robot"
    pkg_share = launch_ros.substitutions.FindPackageShare(package=pkg_name).find(pkg_name)
    default_model_path = os.path.join(pkg_share, 'src/description/my_bot_description.urdf')

    default_rviz_config_path = os.path.join(pkg_share, 'rviz/urdf_config.rviz')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gz_topic = '/model/my_bot'
    joint_state_gz_topic = '/world/empty' + gz_topic + '/joint_state'
    link_pose_gz_topic = gz_topic + '/pose'
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
    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
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
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
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
        # arguments=[
        #     '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        #     joint_state_gz_topic + '@sensor_msgs/msg/JointState[gz.msgs.Model',
        #     link_pose_gz_topic + '@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        #     link_pose_gz_topic + '_static@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        #     gz_topic + '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
        #     gz_topic + '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry',
        # ],
        # remappings=[
        #     (joint_state_gz_topic, '/joint_states'),
        #     (link_pose_gz_topic, '/tf_2'),
        #     (link_pose_gz_topic + '_static', '/tf_static'),
        # ],
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
        #joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node,
        gz_sim,
        spawn,
        ros_gz_bridge_node,
        Node(
            package=pkg_name,
            executable='rot_robot',
            name='rot_robot',
        ),
    ])