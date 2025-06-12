from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch import LaunchDescription
import yaml 
import os 


def generate_launch_description():
    pkg_pirobot_mapping = get_package_share_directory('pirobot_mapping')
    conf_path = os.path.join(
        pkg_pirobot_mapping, 'config', 'config.yaml')
    
    with open(conf_path, 'r') as f:
        mapping_params = yaml.safe_load(f)

    

    mapping_node = Node(
        package="pirobot_mapping",
        executable="pirobot_mapping_node",
        output='screen',
        parameters=[
            mapping_params
        ]
    )


    vicon_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('vicon_receiver'), 'launch', 'client.launch.py')
        )
    )

    ld = LaunchDescription()
    ld.add_action(mapping_node)
    ld.add_action(vicon_launch)
    return ld
