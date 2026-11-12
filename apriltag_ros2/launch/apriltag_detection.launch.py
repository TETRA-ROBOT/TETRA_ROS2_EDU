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
                # image_transport 는 이미지 토픽과 같은 네임스페이스의 camera_info 를 구독하므로 전체 이름으로 리맵.
                # camera_info_fix 노드(tetra_bringup)가 보정값 NaN 을 보완해서 재발행하는 토픽을 사용
                ('/camera/color/camera_info', '/camera/color/camera_info_fixed'),
            ],
        ),
    ])

