import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace


def generate_launch_description():

    ld = LaunchDescription()

    # purethermal2
    node = Node(
        package = 'flir_lepton_purethermal2',
        namespace = 'flir_lepton',
        name = 'purethermal2',
        executable = 'purethermal2_node',
    )
    ld.add_action(node)

    # thermal_image_processor_node
    config = os.path.join(
        get_package_share_directory('flir_lepton_data_processor'), 'config', 'params.yaml')
    node = Node(
            package = 'flir_lepton_data_processor',
            namespace = 'flir_lepton',
            name = 'thermal_image_processor',
            executable = 'thermal_image_processor_node',
            parameters = [config]
        )
    ld.add_action(node)

    return ld