"""Single car omnet module launch file"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    realsense_pkd_dir = get_package_share_directory('realsense2_camera')

    # realsense cmd with ifcondition to enable camera
    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_pkd_dir, 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_depth': 'true',
            'enable_color': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'infra_rgb': 'false',
            'enable_fisheye1': 'true',
            'enable_fisheye2': 'true',
            'enable_confidence': 'true',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'enable_pose': 'true',
            'pointcloud.enable': 'false',
            'enable_sync': 'false',
            'align_depth.enable': 'false',
            'colorizer.enable': 'false',
            'initial_reset': 'false',
            'allow_no_texture_points': 'false',
            'ordered_pc': 'false',
            'decimation_filter.enable': 'false',
            'color_fps': '1',
            'depth_fps': '1',
        }.items()
    )

    # Launch Description
    launch_description = LaunchDescription()

    launch_description.add_action(realsense_cmd)

    return launch_description
