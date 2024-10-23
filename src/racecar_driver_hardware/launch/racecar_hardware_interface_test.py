import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.event_handlers import OnProcessStart

def generate_launch_description():
    # Get the package directory
    sub_package_name = 'racecar_hardware_interface'  # Replace with your package name
    package_dir = get_package_share_directory(sub_package_name)

    # Declare the path to your robot's URDF/Xacro file
    xacro_file = os.path.join(package_dir, 'urdf', 'main.xacro')

    # Use the xacro command to process the file and generate the URDF
    robot_description_content = Command(['xacro ', xacro_file])

    # Wrap the robot_description in ParameterValue with value_type=str
    robot_description = {'robot_description': ParameterValue(robot_description_content, value_type=str)}
    
    #controller configuration file
    controller_params_file = os.path.join(package_dir, 'config', 'racecar_controllers.yaml')


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

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
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