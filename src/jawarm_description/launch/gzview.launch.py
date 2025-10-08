from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_path = os.path.join(get_package_share_directory('jawarm_description'))
    urdf_file = os.path.join(pkg_path, 'urdf', 'jaw.urdf')
    world_file = os.path.join(pkg_path, 'gazebo', 'arm.world')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
            ),
            launch_arguments={'gz_args': world_file}.items()
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{"robot_description": open(urdf_file).read()}]
        ),
        Node(
            package='ros_gz_sim',
            executable='create',
            arguments=['-world', 'default', '-file', urdf_file, '-name', 'jaw_robot'],
            output='screen'
        )
    ])
