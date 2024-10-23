from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ls01g',
            executable='ls01g',
            name='ls01g_node',
            output='screen',
            parameters=[
                {'serial_port': '/dev/laser'},
                {'laser_link': 'laser_link'},
                {'angle_disable_min': -1.0},
                {'angle_disable_max': -1.0},
                {'zero_as_max': False},
                {'min_as_zero': False},
                {'inverted': False},
                #{'log_level': 'DEBUG'}
            ]
        ),
    ])
