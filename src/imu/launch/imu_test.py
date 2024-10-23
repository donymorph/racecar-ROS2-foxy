from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu',
            executable='imu',
            name='imu',
            output='screen',
            parameters=[
                {'port': '/dev/imu'},
                {'model': 'dony_imu_01'},
                {'baud': 115200},
            ]
        )
    ])
