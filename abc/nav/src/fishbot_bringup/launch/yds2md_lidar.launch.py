"""Start a YDS2MD-1 with the YDLIDAR S2 compatible profile.

The protocol match is provisional until verified against live scan data.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    use_wifi = LaunchConfiguration('use_wifi')
    port = LaunchConfiguration('port')
    tcp_port = LaunchConfiguration('tcp_port')

    return LaunchDescription([
        DeclareLaunchArgument('use_wifi', default_value='true',
                              description='true for the FishBot Wi-Fi radar adapter'),
        DeclareLaunchArgument('port', default_value='/tmp/tty_laser',
                              description='USB serial device, or /tmp/tty_laser for Wi-Fi'),
        DeclareLaunchArgument('tcp_port', default_value='8889',
                              description='TCP listening port for the Wi-Fi radar adapter'),
        Node(
            package='ros_serial2wifi',
            executable='tcp_server',
            name='radar_tcp_bridge',
            condition=IfCondition(use_wifi),
            parameters=[{'tcp_port': ParameterValue(tcp_port, value_type=int),
                         'serial_port': '/tmp/tty_laser'}],
            output='screen',
        ),
        TimerAction(period=2.0, actions=[Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            parameters=[{
                'port': port,
                'frame_id': 'laser_frame',
                'baudrate': 115200,
                'lidar_type': 1,
                'device_type': 0,
                'sample_rate': 3,
                'isSingleChannel': True,
                'intensity': False,
                'support_motor_dtr': False,
                'range_min': 0.12,
                'range_max': 8.0,
                'angle_min': -180.0,
                'angle_max': 180.0,
                'auto_reconnect': True,
            }],
            output='screen',
        )]),
    ])
