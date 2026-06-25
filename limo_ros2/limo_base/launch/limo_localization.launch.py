"""
Phase 2 — Localisation sur carte existante
===========================================
Lancer ce fichier pour les sessions d'apprentissage :
    ros2 launch limo_base limo_localization.launch.py \
        map:=$HOME/torch_ros/maps/my_map.yaml

Le robot doit être positionné au point d'origine de la carte (position initiale
de la phase de cartographie). AMCL s'initialise automatiquement à (0, 0, 0).
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name_arg       = LaunchConfiguration('port_name',  default='ttylimo')
    odom_frame_arg      = LaunchConfiguration('odom_frame', default='odom')
    base_link_frame_arg = LaunchConfiguration('base_frame', default='base_link')

    map_arg = DeclareLaunchArgument(
        'map',
        default_value=os.path.expanduser('~/torch_ros/maps/my_map.yaml'),
        description='Chemin vers le fichier YAML de la carte',
    )
    map_file = LaunchConfiguration('map')

    pkg_share = get_package_share_directory('limo_base')
    amcl_params_file = os.path.join(pkg_share, 'config', 'amcl_params.yaml')

    remapping = [
        ('odom', '/wheel/odom'),
    ]

    limo_base_node = Node(
        package='limo_base',
        executable='limo_base',
        output='screen',
        name='limo_base_node',
        emulate_tty=True,
        parameters=[{
            'port_name':   port_name_arg,
            'odom_frame':  odom_frame_arg,
            'base_frame':  base_link_frame_arg,
            'pub_odom_tf': True,
            'use_mcnamu':  False,
        }],
        remappings=remapping,
    )

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file, 'use_sim_time': False}],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_file],
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart':    True,
            'node_names':   ['map_server', 'amcl'],
        }],
    )

    return LaunchDescription([
        map_arg,
        limo_base_node,
        map_server_node,
        amcl_node,
        lifecycle_manager,
    ])
