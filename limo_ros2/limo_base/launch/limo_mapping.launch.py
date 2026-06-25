"""
Phase 1 — Cartographie
======================
Lancer ce fichier pour construire la carte de la salle :
    ros2 launch limo_base limo_mapping.launch.py

Télé-opérer le robot pour couvrir toute la salle, puis sauvegarder la carte :
    ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap \
        "{name: {data: '$HOME/torch_ros/maps/my_map'}}"

La carte produit deux fichiers : my_map.yaml  et  my_map.pgm
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name_arg       = LaunchConfiguration('port_name',  default='ttylimo')
    odom_frame_arg      = LaunchConfiguration('odom_frame', default='odom')
    base_link_frame_arg = LaunchConfiguration('base_frame', default='base_link')

    pkg_share = get_package_share_directory('limo_base')
    slam_params_file = os.path.join(pkg_share, 'config', 'slam_mapping_params.yaml')

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

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file],
    )

    return LaunchDescription([
        limo_base_node,
        slam_node,
    ])
