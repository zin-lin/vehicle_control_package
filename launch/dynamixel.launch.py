"""
Author: Zin Lin Htun
class: Launch
"""

# import necessaries
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# import descriptions
from launch_ros.actions import Node

# constants
PKG_SRC = 'vehicle_control_package'
PARAMETERS_SRC = 'vehicle_control_package/parameters'

def generate_launch_description():
    # get bridge
    bridge_file = os.path.join(get_package_share_directory(PKG_SRC),PARAMETERS_SRC, "bridge_parameters.yaml")

    dynamixel_vcu_node = Node(
        package='vehicle_control_package',
        executable='dynamixel',
        output='both', # both means both log files and terminal
    )


    # empty launch_des
    launch_description = LaunchDescription()

    # launch extra components
    launch_description.add_action(dynamixel_vcu_node) # adding vcu control unit

    return launch_description
