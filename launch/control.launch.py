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

    vcu_node = Node(
        package='vehicle_control_package',
        executable='vehicle_control',
        output='both', # both means both log files and terminal
    )

    imu_node = Node(
        package='motion_package',
        executable='imu',
        output='both', # both means both log files and terminal
    )

    odometry_node = Node(
        package='motion_package',
        executable='odometry',
        output='both',  # both means both log files and terminal
    )


    # empty launch_des
    launch_description = LaunchDescription()

    # launch extra components
    launch_description.add_action(vcu_node) # adding vcu control unit
    launch_description.add_action(imu_node) # adding vcu control unit
    launch_description.add_action(odometry_node) # adding vcu control unit


    return launch_description
