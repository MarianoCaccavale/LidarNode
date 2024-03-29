import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():    

    lidar_config_file = "Lidar_node.yaml"
    ground_config_file = "Ground_removal.yaml"

    lidar_config=os.path.join(get_package_share_directory('lidar_node'),"config", lidar_config_file)
    ground_config=os.path.join(get_package_share_directory('lidar_node'),"config", ground_config_file)

    lidar_node= Node(
        package='lidar_node',
        executable='lidar_node',
        name='lidar_node',
        parameters=[
            {"lidar_yaml_config_path": lidar_config},
            {"ground_yaml_config_path": ground_config}
        ],
    )

    return LaunchDescription([lidar_node])

