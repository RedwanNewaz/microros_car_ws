from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
import yaml 
import os 
import sys


def generate_launch_description():
    pkg_create3_cbo = get_package_share_directory('bow_planner')
    env_path = os.path.join(
        pkg_create3_cbo, 'config', 'param.yaml')
    
    with open(env_path, 'r') as f:
        cbo_params = yaml.safe_load(f)

    

    cbo_node = Node(
        package="bow_planner",
        executable="bow_planner_node",
        output='screen',
        parameters=[
            cbo_params
        ]
    )
    ld = LaunchDescription()
    ld.add_action(cbo_node)
    return ld
