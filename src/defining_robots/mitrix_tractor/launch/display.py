from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.conditions import IfCondition
# from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, 
    # FindExecutable, 
    PathJoinSubstitution, 
    LaunchConfiguration, 
    # PathSubstitution
)
from launch.actions import (
    # DeclareLaunchArgument,
    IncludeLaunchDescription,
    # ExecuteProcess,
    # RegisterEventHandler
)
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import os
from launch.launch_description_sources import PythonLaunchDescriptionSource
# from ament_index_python.packages import get_package_share_directory
# from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
# from launch.actions import RegisterEventHandler
# from launch.event_handlers import OnProcessExit
# from launch.launch_description_sources import PythonLaunchDescriptionSource
# from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
 
# from launch_ros.actions import Node
# from launch_ros.substitutions import FindPackageShare
import xacro
# def generate_launch_description():
#     return LaunchDescription([

    # ])

def generate_launch_description():
    # world_file = LaunchConfiguration('world_file', default='')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_ros_mitrix_tractor_demos = get_package_share_directory('mitrix_tractor')

    # urdf_path = PathJoinSubstitution([
    #     FindPackageShare('mitrix_tractor'),
    #     'urdf',
    #     'mitrix_tractor.urdf'
    # ])

    # robot_description_content = ParameterValue(
    #     Command(['xacro ', urdf_path]),
    #     value_type=str
    # )
    # robot_description = {"robot_description": robot_description_content}
    

    urdf = Node(        
            IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
                launch_arguments={
                    'urdf_package': 'turtlebot3_description',
                    'urdf_package_path': PathJoinSubstitution(['urdf', 'turtlebot3_burger.urdf'])
                }.items()
        ))

    # Parse robot description from xacro
    robot_description_file = os.path.join(pkg_ros_mitrix_tractor_demos, 'models', 'rrbot.xacro')
    robot_description_config = xacro.process_file(
        robot_description_file
    )
    robot_description = {'robot_description': robot_description_config.toxml()}

    robot_state_publisher = Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="both",
                parameters=[robot_description],
            )
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        # output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'mitrix', '-allow_renaming', 'true'],#, '-s', '-v', '4'],
    )

    # Gazebo Sim
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                       'launch',
                                       'gz_sim.launch.py'])]),
            # launch_arguments=[('gz_args', [' -r empty.sdf'])]),
            launch_arguments=[('gz_args', [' -s -r -v 4 empty.sdf']),
                              ('ign_args', [' -s -v 4'])]),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=gz_spawn_entity,
        #         on_exit=[joint_state_broadcaster_spawner],
        #     )
        # ),
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=joint_state_broadcaster_spawner,
        #         on_exit=[diff_drive_base_controller_spawner],
        #     )
        # ),
        gz_spawn_entity,
        robot_state_publisher
        ])
