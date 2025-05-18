from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[{
                'fcu_url': 'serial:///dev/ttyAMA1:57600',
                'gcs_url': 'udp://:14550@',
                'target_system_id': 1,
                'target_component_id': 1
            }]
        )
    ])
