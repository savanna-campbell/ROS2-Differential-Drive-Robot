import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('my_robot_localization'),
        'config',
        'ekf.yaml'
    )

    return LaunchDescription([
        Node(
            package='my_robot_controller',
            executable='arduino_bridge',
            name='arduino_bridge',
            output='screen',
        ),
        Node(
            package='actions_py',
            executable='behavior_server',
            name='behavior_server',
            output='screen',
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[config],
        ),
    ])