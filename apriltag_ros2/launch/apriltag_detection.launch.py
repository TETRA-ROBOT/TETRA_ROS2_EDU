from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            output='screen',
            parameters=[
                {'camera_frame': 'camera_color_optical_frame'},
                {'tag_family': 'tag36h11'},
                {'size': 0.1}  # Adjust to your tag's size in meters
            ],
            remappings=[
                ('/image_rect', '/camera/color/image_raw'), # /camera/image_raw
                ('/camera_info', '/camera/color/camera_info'), # /camera/camera_info
            ],
        ),
    ])

