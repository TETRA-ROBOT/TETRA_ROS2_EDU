from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='laser_line_extraction_ros2',
            executable='line_extraction_node',
            name='line_extractor_node',
            output='screen',
            parameters=[{
                'frame_id': 'camera1_link',
                'scan_topic': '/scan_filtered',
                'publish_markers': True,
                'bearing_std_dev': 1e-5,
                'range_std_dev': 0.012,
                'least_sq_angle_thresh': 0.0001,
                'least_sq_radius_thresh': 0.0001,
                'max_line_gap': 0.2,          # Maximum distance between two points in the same line (m)
                'min_line_length': 0.15,      # Lines shorter than this are not published (m)
                'min_range': 0.05,            # Minimum range
                'max_range': 1.0,             # Maximum range
                'min_split_dist': 0.04,       # Split step threshold (m)
                'outlier_dist': 0.06,         # Outlier distance (m)
                'min_line_points': 5          # Minimum number of points in a line
            }]
        )
    ])
