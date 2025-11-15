from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    parameter = [{'use_sim_time': True}]
    return LaunchDescription([
        Node(
            package='your_solution',
            executable='calc_error',
            parameters=parameter
        ),
        Node(
            package='your_solution',
            executable='tf_broadcaster',
            parameters=parameter
        ),
        Node(
            package='huskybot_cv',
            executable='huskybot_cv',
            parameters=parameter,
        ),
        Node(
            package='sim_node',
            executable='keyboard_controls',
        ),
        IncludeLaunchDescription(
            
            PathJoinSubstitution([
                FindPackageShare('sim_node'),
                'launch',
                'sim_node_launch.py'
            ]),
            launch_arguments={
                'cv_exposure': '0.8',
                'cpu_sim': 'true',
            }.items()
        )
    ])