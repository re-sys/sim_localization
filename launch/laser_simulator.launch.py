from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Get the package share directory
    pkg_path = get_package_share_directory('sim_localization')
    
    # Path to the parameters file
    params_file = os.path.join(pkg_path, 'config', 'laser_simulator_params.yaml')
    
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': os.path.join(
                get_package_share_directory('sim_localization'),
                'maps/basketball_court/map.yaml'),
            'topic_name': 'map',
            'frame_id': 'map'
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_mapper',
        parameters=[{
            'node_names': ['map_server'],
            'autostart': True,
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen'
    )

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(pkg_path, 'rviz', 'sim_localization.rviz')]
        ),
        Node(
            package='sim_localization',
            executable='laser_simulator',
            name='laser_simulator',
            parameters=[params_file]
        ),
        map_server_node,
        TimerAction(
            period=3.0,  # Wait 3 seconds before starting lifecycle manager
            actions=[lifecycle_manager]
        )
    ])