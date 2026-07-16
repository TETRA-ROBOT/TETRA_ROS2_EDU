#!/usr/bin/env python3
"""
line_docking_detector launch 파일
LiDAR V-shape 라인 검출 + TF 발행 노드를 실행합니다.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('line_docking_detector')
    params_file = os.path.join(pkg_dir, 'config', 'params.yaml')

    return LaunchDescription([
        Node(
            package='line_docking_detector',
            executable='line_detector_node',
            name='line_detector_node',
            output='screen',
            parameters=[params_file],
        ),
    ])
