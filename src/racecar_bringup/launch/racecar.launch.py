import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get the package directory
    package_name = 'racecar_bringup'  # Replace with your package name
    package_dir = get_package_share_directory(package_name)

    # Declare the path to your robot's URDF/Xacro file
    xacro_file = os.path.join(package_dir, 'urdf', 'main.xacro')

    # Use the xacro command to process the file and generate the URDF
    robot_description_content = Command(['xacro ', xacro_file])

    # Wrap the robot_description in ParameterValue with value_type=str
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    #controller configuration file
    controller_params_file = os.path.join(package_dir, 'config', 'racecar_controllers.yaml')

    robot_localization_config = os.path.join(package_dir, 'config', 'EKF.yaml')

    twist_mux_config = os.path.join(package_dir, 'config', 'twist_mux.yaml')

    # Controller Manager is the main component in the ros2_control framework. 
    # It manages lifecycle of controllers, access to the hardware interfaces and offers services to the ROS-world.
    # documentation: https://control.ros.org/rolling/doc/ros2_control/controller_manager/doc/userdoc.html#
    # https://github.com/ros-controls/ros2_control
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controller_params_file],
        output={
            "stdout": "screen",
            "stderr": "screen",
        },

    )
    # This package contains the Robot State Publisher, a node and a class to publish the state of a robot to tf2.
    # https://github.com/ros/robot_state_publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='both',
        parameters=[robot_description]
    )

    # ackermann_steering_controller spawner
    # http://wiki.ros.org/ackermann_steering_controller
    # https://github.com/harderthan/ackermann-steering-controller-ros2/tree/foxy-devel
    ackermann_steering_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["ackermann_steering_controller", "--controller-manager", "/controller_manager"],
    )
    # delayed_ackermann_controller is used to ensure that the ackermann_steering_controller node is started 
    # only after the controller_manager node has successfully started.
    delayed_ackermann_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[ackermann_steering_controller],
        )
    )
    # The broadcaster reads all state interfaces and reports them on /joint_states and /dynamic_joint_states
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
    )
    # delayed_joint_state_broadcaster is used to ensure that the ackermann_steering_controller node is started 
    # only after the controller_manager node has successfully started.
    delayed_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster],
        )
    )
    # optional, manually manipulate joints
    joint_state_publisher_node_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )


    # rviz - 3D visualizer for the Robot Operating System (ROS) framework.
    rviz_config_file = os.path.join(package_dir, 'config', 'racecar.rviz')
    rviz_config2_file = os.path.join('/home/car/racecarROS2/src/racecar_bringup/config', 'racecar.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config2_file],
    )
    # ls01g model lidar launch
    lidar_node = Node(
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
        )
    # Estimation of 2D odometry based on planar laser scans
    # https://github.com/MAPIRlab/rf2o_laser_odometry
    rf2o_laser_node = Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,
                'base_frame_id': 'base_link',
                'odom_frame_id': 'odom_rf2o',
                'init_pose_from_topic': '',
                'freq': 10.0,  # Increased frequency for better odometry
                'verbose': False
            }],
        )
    # imu launch
    imu_node = Node(
            package='imu',
            executable='imu',
            name='imu',
            output='screen',
            parameters=[
                {'port': '/dev/imu'},
                {'model': 'dony_imu_01'},
                {'baud': 115200},
                {'frame_id': 'imu_link'},
                {'delay': 0.0}
            ]
        )
    # EKF and UKF: robot_localization is a package of nonlinear state estimation nodes.
    # https://github.com/cra-ros-pkg/robot_localization
    # Documentation: http://docs.ros.org/en/melodic/api/robot_localization/html/index.html
    robot_localization = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            #parameters=[os.path.join(get_package_share_directory("robot_localization"), 'params', 'ekf.yaml')],
            parameters=[robot_localization_config],
            remappings=[('imu', 'imu/data'),
                        ('gps/fix', 'gps/fix')]
                        #('odometry/filtered', 'odometry/global')]
        )
    # optional: at the beginning rviz throws erros messages for a while if you dont add this. 
    map2odomlink=Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0.15', '0', '0', '0', 'map', 'odom'],
        )

    #The main node of this package is twist_mux, which provides a multiplexer for geometry_msgs::Twist messages. 
    #It takes N input twist topics and outputs the messages from a single one. 
    #http://wiki.ros.org/twist_mux
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        remappings=[('/cmd_vel_out', '/ackermann_steering_controller/cmd_vel_unstamped')],
        parameters=[twist_mux_config]
    )
    # Launch description
    ld = LaunchDescription()

    # Add nodes to the launch description
    ld.add_action(robot_state_publisher_node)
    ld.add_action(controller_manager)
    ld.add_action(delayed_joint_state_broadcaster)
    ld.add_action(delayed_ackermann_controller)
    #ld.add_action(joint_state_publisher_node_gui)
    ld.add_action(rviz_node)
    ld.add_action(lidar_node)
    ld.add_action(imu_node)
    ld.add_action(rf2o_laser_node)
    ld.add_action(robot_localization)
    ld.add_action(map2odomlink)
    ld.add_action(twist_mux_node)
    return ld
