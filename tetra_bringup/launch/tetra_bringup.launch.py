#!/usr/bin/env python3

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


# this is the function launch  system will look for
def generate_launch_description():

    # tetra Motor Driver Board
    tetra_node = Node(
        package='tetra', 
        executable='tetra',
        output='screen',
        parameters=[
            {"m_bEKF_option": True} #default: False
        ]
    )
    
    # EKF Localization
    ekf_localization_node= Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory("tetra_bringup"), 'params', 'ekf.yaml')],
            arguments=['--ros-args', '--log-level', 'error']
            #remappings=[('/odometry/filtered', '/odom')],
    )
    
    # tetra_interface Board
    tetra_interface_node = Node(
        package='tetra_interface', 
        executable='tetra_interface',
        output='screen',
        parameters=[
            {"m_bConveyor_option": False},
            {"m_bUltrasonic_option": False}
        ]
    )
    
    # IMU Sensor
    iahrs_driver_node = Node(
        package='iahrs_driver', 
        executable='iahrs_driver',
        output='screen',
        parameters=[
            {"m_bSingle_TF_option": False} # default: True
        ]
    )
    
    # Joystick
    joy_node = Node(
        package='joy', 
        executable='joy_node',
        output='screen',
        parameters=[
            {"deadzone": 0.05}
        ]
    )
    
    # tetra_URDF
    use_sim_time = DeclareLaunchArgument('use_sim_time', default_value="false")
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description':
                Command([
                    'xacro ',
                    PathJoinSubstitution([
                        FindPackageShare('tetra_description'),
                        'urdf',
                        'tetra.xacro',
                    ]),
                ]),
        }]
    )
    
    # tetra_service
    tetra_service_node = Node(
        package='tetra_service', 
        executable='tetra_service',
        respawn= True,
        output='screen',
        parameters=[
            {"m_dHome_ID": 0}
        ]
    )

    # rosbridge_server 
    rosbridge_server = Node(
        package='rosbridge_server', 
        executable='rosbridge_websocket',
        output='screen',
        parameters=[
            {"port": 9090}
        ]
    )

    # rosapi_node 
    rosapi_node = Node(
        package='rosapi', 
        executable='rosapi_node',
        output='screen',
    )
    
    # Kanavi R270
    kanavi_lidar_node = Node(
        package='kanavi_vl', 
        executable='R270',
        name='r270_node',
        output='screen',
        arguments=[
            '-i', '192.168.0.10', '5000',
            '-m', '224.0.0.5',
            '-fix', 'laser',
            '-topic', 'points2'
        ]
    )
    
    # pointcloud to scan
    pointcloud_to_scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        output='screen',
        remappings=[
            ('cloud_in', '/points2'),
        ],
        parameters=[{
            'target_frame': 'camera1_link',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -1.8415926,   # -135
            'angle_max':  2.8415926,   # 135
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 0.1,
            'range_max': 30.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }]
    )
    
    # create and return launch description object
    return LaunchDescription(
        [
            tetra_node,
            ekf_localization_node,
            tetra_interface_node, 
            # iahrs_driver_node,
            joy_node,
            use_sim_time,
            rsp_node,
            tetra_service_node,
            rosbridge_server,
            rosapi_node,
            # kanavi_lidar_node,
            # pointcloud_to_scan_node,
            
		# USB Camera
		# IncludeLaunchDescription(
		# PythonLaunchDescriptionSource(
		# 	[get_package_share_directory('usb_cam'), '/launch/camera.launch.py']),
		# ),


		# IncludeLaunchDescription(
		# PythonLaunchDescriptionSource(
		# 	[get_package_share_directory('line_tf_publisher'), '/launch/line_tf_publisher.launch.py']),
		# ),
        
        # apriltag_ros 
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('apriltag_ros'), '/launch/apriltag_detection.launch.py']),
		),
			
		# sick_tim_571
		#IncludeLaunchDescription(
		#PythonLaunchDescriptionSource(
		#	[get_package_share_directory('sick_scan_xd'), '/launch/sick_tim_5xx.launch.py']),
		#),

        # autonics lidar
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('lsc_ros2_driver'), '/launch/lsc_ros2_driver_launch.py']),
		),
		
		# cygbot 2D lidar
		#IncludeLaunchDescription(
		#PythonLaunchDescriptionSource(
		#	[get_package_share_directory('cyglidar_d2_ros2'), '/launch/cyglidar.launch.py']),
		#),
		
		# realsense D455
		IncludeLaunchDescription(
		PythonLaunchDescriptionSource(
			[get_package_share_directory('realsense2_camera'), '/launch/rs_launch.py']),
		),

		# Livox MID-360 (3D LiDAR)
		#IncludeLaunchDescription(
		#PythonLaunchDescriptionSource(
		#	[get_package_share_directory('livox_ros_driver2'), '/launch_ROS2/MID360_launch.py']),
		#),

        # laser filter
		# IncludeLaunchDescription(
		# PythonLaunchDescriptionSource(
		# 	[get_package_share_directory('laser_filters'), '/examples/angular_filter_example.launch.py']),
		# ),

        # laser line extraction
		# IncludeLaunchDescription(
		# PythonLaunchDescriptionSource(
		# 	[get_package_share_directory('laser_line_extraction_ros2'), '/launch/example.launch.py']),
		# ),
        
        ]
    )
