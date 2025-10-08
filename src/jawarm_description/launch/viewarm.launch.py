from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    urdf_file='/home/ktl/Jaw_roboarm/src/jawarm_description/urdf/jaw.urdf'
    
    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{"use_sim_time": False, "robot_description": open("src/jawarm_description/urdf/jaw.urdf").read()}]
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            output="screen"
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', '/home/ktl/Jaw_roboarm/src/jawarm_description/rviz/viewarm.rviz']
        )
    ])