import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare(package='ddr').find('ddr')
    map_yaml = os.path.join(pkg_share, 'maps', 'my_custom_map.yaml')

    lifecycle_nodes = [
        'map_server',
        'amcl'
    ]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[{
            'yaml_filename': map_yaml,
            'use_sim_time': True,
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        parameters=[{'use_sim_time':True}]
    )

    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='map_lifecycle_manager',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': lifecycle_nodes
        }]
    )

    return LaunchDescription([
        map_server_node,
        amcl_node,
        lifecycle_manager_node
    ])