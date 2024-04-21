from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
import os
import xacro
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('sf_bot_description')

    xacro_file = os.path.join(share_dir, 'urdf', 'sf_bot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    use_sim_time = LaunchConfiguration('use_sim_time')
    pause_sim = LaunchConfiguration('pause_sim')
    run_control = LaunchConfiguration('run_control')
    
    
    use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='whether to use simulation time or not'
    )


    pause_sim_cmd = DeclareLaunchArgument(
        'pause_sim',
        default_value='true',
        description='whether to pause gazebo sim or not'

    )

    run_controller_cmd = DeclareLaunchArgument(
        'run_control',
        default_value='false',
        description='whether to start balancing controller or not'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    gazebo_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzserver.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': pause_sim
        }.items()
    )

    gazebo_client = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('gazebo_ros'),
                'launch',
                'gzclient.launch.py'
            ])
        ])
    )

    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'sf_bot',
            '-topic', 'robot_description'
        ],
        output='screen'
    )

    
    # balancing_controller_node = Node(
    #     # condition=UnlessCondition(pause_sim),
    #     package='cascade_controller_test',
    #     executable='cas_control_node',
    #     output='screen'
    # )

    return LaunchDescription([
        pause_sim_cmd,
        robot_state_publisher_node,
        gazebo_server,
        gazebo_client,
        urdf_spawn_node,
        # balancing_controller_node
    ])
