# Copyright 2022 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

print("microros_ros2_control_demos")
def generate_launch_description():
    # Launch Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)
    # ign_args = LaunchConfiguration('ign_args', default=["-s", "-v", "4"])
    ign_args = LaunchConfiguration('ign_args', default=["-v", "4"])
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name='xacro')]),
            ' ',
            PathJoinSubstitution(
                [FindPackageShare('microros_ros2_control_demos'),
                 'urdf', 'test_diff_drive.xacro.urdf']
            ),
        ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare('microros_ros2_control_demos'),
            'config',
            'diff_drive_controller.yaml',
        ]
    )

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        # output='screen',
        parameters=[robot_description]
    )

    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        # output='screen',
        arguments=['-topic', 'robot_description', '-name',
                   'diff_drive', '-allow_renaming', 'true', '-v', '4'],
                #    'diff_drive', '-allow_renaming', 'true', '-s', '-v', '4'],
    )
                       
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    diff_drive_base_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'diff_drive_base_controller',
            '--param-file',
            robot_controllers,
            ],
    )

    # Bridge
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        # output='screen'
    )

    return LaunchDescription([
        # Launch gazebo environment
        IncludeLaunchDescription( #[объяснение] лаунч симуляции газебо
            PythonLaunchDescriptionSource(
                [PathJoinSubstitution([FindPackageShare('ros_gz_sim'),
                                        'launch',
                                        'gz_sim.launch.py'])]),
            launch_arguments=[('gz_args', ['--record-topic ".*" -r -v 4 empty.sdf'])]),
            # launch_arguments=[('gz_args', [' -s -r -v 4 empty.sdf']), ('ign_args', [' -s -v 4'])]),
        RegisterEventHandler( #[объяснение] 
            event_handler=OnProcessExit(
                target_action=gz_spawn_entity,
                on_exit=[joint_state_broadcaster_spawner],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state_broadcaster_spawner,
                on_exit=[diff_drive_base_controller_spawner],
            )
        ),
        bridge,
        node_robot_state_publisher,
        gz_spawn_entity,
        # # Launch Arguments
        # DeclareLaunchArgument(
        #     'use_sim_time',
        #     default_value=use_sim_time,
        #     description='If true, use simulated clock'),
        DeclareLaunchArgument(
            'ign_args',
            default_value=ign_args),
    ])
