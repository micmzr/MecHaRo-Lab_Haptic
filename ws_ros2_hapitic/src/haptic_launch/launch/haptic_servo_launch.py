from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder
import os
import yaml

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def launch_setup(context, *args, **kwargs):
    # Command-line arguments
    robot = LaunchConfiguration("robot")
    device = LaunchConfiguration("device")

    config_params = {
        "config_servo": ParameterBuilder("haptic_launch")
        .yaml("config/"+f"{robot.perform(context)}"+".yaml")
        .to_dict()
    }

    haptic_node = Node(
        package='haptic_servo',
        executable=f"{device.perform(context)}"+'_servo',
        parameters=[
            config_params["config_servo"],
        ],
        arguments=['--log-level', 'info']
    )

    nodes_to_start = [haptic_node]
    return nodes_to_start

def generate_launch_description():

    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot",
            default_value="hcr3a",
            description="Config file describing the robot.",
            choices=["hcr3a", "panda"],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "device",
            default_value="joy",
            description="Device for controling the robot.",
            choices=["joy", "phantom"],
        )
    )

    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=launch_setup)])

    