import os
import launch
import launch_ros

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_name_arg = LaunchConfiguration('port_name',  default='ttylimo')
    odom_frame_arg = LaunchConfiguration('odom_frame', default='odom')
    base_link_frame_arg = LaunchConfiguration('base_frame', default='base_link')

    pkg_share = get_package_share_directory('limo_base')
    ekf_params_file = os.path.join(pkg_share, 'config', 'ekf_params.yaml')

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
            'pub_odom_tf': False,   # EKF publie le TF odom→base_link
            'use_mcnamu':  False,
        }],
        remappings=remapping,
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_node',
        output='screen',
        parameters=[ekf_params_file],
    )

    return LaunchDescription([
        limo_base_node,
        ekf_node,
    ])
