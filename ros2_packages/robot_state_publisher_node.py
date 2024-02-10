import os
from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file_path = os.path.join(
        get_package_share_directory('ros2_packages'),
        'urdf',
        'vehicle.urdf'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': False}],
            arguments=[urdf_file_path]
        )
    ])

if __name__ == '__main__':
    generate_launch_description()
