import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    limo_base_share = get_package_share_directory('limo_base')
    ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')

    use_mcnamu_arg = DeclareLaunchArgument(
        'use_mcnamu', default_value='false',
        description='Switch the vehicle to mecanum mode')

    limo_base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(limo_base_share, 'launch', 'limo_base.launch.py')),
        launch_arguments={'use_mcnamu': LaunchConfiguration('use_mcnamu')}.items())

    ydlidar_node = Node(
        package='ydlidar_ros2_driver',
        executable='ydlidar_ros2_driver_node',
        name='ydlidar_ros2_driver_node',
        output='screen',
        emulate_tty=True,
        parameters=[os.path.join(ydlidar_share, 'params', 'TminiPro.yaml')],
    )

    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_camera',
        arguments=['0.105', '0', '0.1', '0', '0', '0', '1', 'base_link', 'camera_link'],
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_imu',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'imu_link'],
    )

    static_tf_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_laser',
        arguments=['0.105', '0', '0.08', '0', '0', '0', '1', 'base_link', 'laser_frame'],
    )

    return LaunchDescription([
        use_mcnamu_arg,
        limo_base_launch,
        ydlidar_node,
        static_tf_camera,
        static_tf_imu,
        static_tf_laser,
    ])
