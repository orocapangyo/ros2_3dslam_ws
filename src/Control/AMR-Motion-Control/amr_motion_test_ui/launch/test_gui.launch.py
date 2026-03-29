"""Launch the PyQt5 GUI version of the motion test UI."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        Node(
            package='amr_motion_test_ui',
            executable='gui_node',
            name='amr_motion_test_gui',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
        ),
    ])
