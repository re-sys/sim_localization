from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import TimerAction

def generate_launch_description():
    # Get the package share directory
    pkg_path = get_package_share_directory('sim_localization')
    
    # Map server node
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': os.path.join(pkg_path, 'maps/basketball_court/map.yaml'),
            'topic_name': 'map',
            'frame_id': 'map'
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
        output='screen'
    )

    # Lifecycle manager to activate map server
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

    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_path, 'rviz', 'sim_localization.rviz')]
    )

    # Static transform publisher for map frame (optional, in case no other node provides it)
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        output='screen'
    )

    return LaunchDescription([
        map_server_node,
        TimerAction(
            period=2.0,  # Wait 2 seconds before starting lifecycle manager
            actions=[lifecycle_manager]
        ),
        static_tf_node,
        rviz_node,
    ]) 