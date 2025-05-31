from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    parameter_file = os.path.join(
        get_package_share_directory('final_project'),
        'config',
        'waypoint_params.yaml'
    )
    return LaunchDescription([
        Node(
            package='final_project',
            executable='reach_target',
            name='robot_target_client',
            parameters=[parameter_file]
        )
    ])
