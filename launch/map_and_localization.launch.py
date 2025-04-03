import os
from launch import LaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = FindPackageShare(package='ddr').find('ddr')
    map_yaml = os.path.join(pkg_share, 'maps', 'my_custom_map.yaml')
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    ekf_config = os.path.join(pkg_share, 'config', 'ekf.yaml')

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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time':True}],
        remappings=[
            ('/odom','/ekf_odom'),
            ('/map','/ekf_map')
        ]
    )

    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config],
        remappings=[(
            'cmd_vel_out', 'diff_cont/cmd_vel_unstamped'
        )]
    )



    return LaunchDescription([
        twist_mux_node,
        ekf_node,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])