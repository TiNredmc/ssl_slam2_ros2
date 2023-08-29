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
            {'scan_period': 0.05},
            {'vertical_angle': 2.0},
            {'max_dis': 5.0},
            {'map_resolution': 0.02},
            {'min_dis': 0.4},
            {'skip_frames': 1},
            {'map_path': home},   
        ],
        output='screen',
    )
    
    odom_est_localize_node = launch_ros.actions.Node(
        package='ssl_slam2',
        executable='ssl_slam2_odom_estimation_localization_node',
        parameters=[
            {'scan_period': 0.05},
            {'vertical_angle': 2.0},
            {'max_dis': 5.0},
            {'map_resolution': 0.02},
            {'min_dis': 0.4},
            {'skip_frames': 1},
            {'map_path': home},   
        ],
        output='screen',
    )


    return launch.LaunchDescription([
        laser_proc_node,
        odom_est_localize_node,
    ])