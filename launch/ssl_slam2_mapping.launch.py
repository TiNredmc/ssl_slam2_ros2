import os

import launch
import launch_ros.actions

from ament_index_python.packages import get_package_share_directory

home = os.path.expanduser('~')

def generate_launch_description():

    laser_proc_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_laser_processing_node',
        parameters=[
            {'scan_period': 0.03},
            {'vertical_angle': 2.0},
            {'max_dis': 8.0},
            {'map_resolution': 0.005},
            {'min_dis': 0.3},
            {'skip_frames': 1},
            {'map_path': home},   
        ],
        output='screen',
    )
    
    odom_est_map_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_odom_estimation_mapping_node',
        parameters=[
            {'scan_period': 0.03},
            {'vertical_angle': 2.0},
            {'max_dis': 8.0},
            {'map_resolution': 0.005},
            {'min_dis': 0.3},
            {'skip_frames': 1},
            {'map_path': home},
            {'publish_odom_tf': True}   
        ],
        output='screen',
    )
    
    map_optz_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_map_optimization_node',
        parameters=[
            {'scan_period': 0.03},
            {'vertical_angle': 2.0},
            {'max_dis': 8.0},
            {'map_resolution': 0.005},
            {'min_dis': 0.3},
            {'skip_frames': 1},
            {'map_path': home},
            {'min_map_update_distance' : 0.5},
            {'min_map_update_angle': 20.0},
            {'min_map_update_frame': 8.0},
        ],
        output='screen',
    )

    return launch.LaunchDescription([
        laser_proc_node,
        odom_est_map_node,
        map_optz_node,
    ])
