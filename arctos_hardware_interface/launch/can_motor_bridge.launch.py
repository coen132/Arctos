from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    """""
    Can frame encoder/decoder for stepper boards Servo42D/57D
    """""

    launch_arguments = []
    arctos_hardware_interface_share_dir = Path(get_package_share_directory("arctos_hardware_interface"))

    motor_can_recv_topic_arg = DeclareLaunchArgument(
        name="motor_can_recv_topic",
        default_value="/from_motor_can_bus",
        description="Topic to use for receiving from the motor network CAN messages",
    )
    launch_arguments.append(motor_can_recv_topic_arg)

    motor_can_send_topic_arg = DeclareLaunchArgument(
        name="motor_can_send_topic",
        default_value="/to_motor_can_bus",
        description="Topic to use for sending to the motor network CAN messages",
    )
    launch_arguments.append(motor_can_send_topic_arg)

    # Nodes, executables
    launch_executables = []  # Hold all executables which are started at the start of the launch file

    # Launch can socket bridge
    bringup_can_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(arctos_hardware_interface_share_dir / "launch" / "can_interface.launch.py")),
    )
    launch_executables.append(bringup_can_launch)




