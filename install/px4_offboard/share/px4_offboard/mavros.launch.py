# Create or edit mavros_px4.launch.py in the launch directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mavros',
            executable='mavros_node',
            output='screen',
            parameters=[
                {'fcu_url': 'udp://:14540@127.0.0.1:14557'},  # Adjust this URL as needed
                {'gcs_url': 'udp://@127.0.0.1:14550'},  # Optional: Connect MAVROS to a GCS like QGroundControl
                {'target_system_id': 1},  # System ID of the PX4
                {'target_component_id': 1}  # Component ID for the autopilot
            ]
        )
    ])
