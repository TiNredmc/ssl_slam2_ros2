import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    laser_proc_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_laser_processing_node',
        parameters=[
            {'scan_period': 0.1},
            {'vertical_angle': 2.0},
            {'max_dis': 9.0},
            {'map_resolution': 0.02},
            {'min_dis': 0.1},
            {'skip_frames': 1},
            {'map_path': '/home/tinlethax/'},   
        ],
        output='screen',
    )
    
    odom_est_map_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_odom_estimation_mapping_node',
        parameters=[
            {'scan_period': 0.1},
            {'vertical_angle': 2.0},
            {'max_dis': 9.0},
            {'map_resolution': 0.02},
            {'min_dis': 0.1},
            {'skip_frames': 1},
            {'map_path': '/home/tinlethax/'},   
        ],
        output='screen',
    )
    
    map_optz_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_map_optimization_node',
        parameters=[
            {'scan_period': 0.1},
            {'vertical_angle': 2.0},
            {'max_dis': 9.0},
            {'map_resolution': 0.02},
            {'min_dis': 0.1},
            {'skip_frames': 1},
            {'map_path': '/home/tinlethax/'},
            {'min_map_update_distance' : 1.0},
            {'min_map_update_angle': 30.0},
            {'min_map_update_frame': 8.0},
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        laser_proc_node,
        odom_est_map_node,
        map_optz_node,
    ])