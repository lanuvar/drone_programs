from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='red_drone_demo',
            executable='mavlink_iface',
            name='mavlink_iface',
            output='screen',
            parameters=[{'conn_str': 'udp:127.0.0.1:14550'}]
        ),
        Node(
            package='red_drone_demo',
            executable='red_finder',
            name='red_finder',
            output='screen',
            parameters=[{'image_topic': '/camera/image_raw', 'min_area': 1200}]
        ),
        Node(
            package='red_drone_demo',
            executable='drone_brain',
            name='drone_brain',
            output='screen'
        ),
    ])