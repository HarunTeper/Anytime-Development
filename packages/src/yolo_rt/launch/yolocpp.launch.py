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
    pkg_dir = get_package_share_directory('yolo_rt')
    realsense_pkd_dir = get_package_share_directory('realsense2_camera')

    weights_path = os.path.join(pkg_dir, 'weights')

    print('weights_path: ', weights_path)

    enable_camera = LaunchConfiguration('enable_camera', default='true')
    threading_type = LaunchConfiguration('threading_type', default='single')
    use_waitable = LaunchConfiguration('use_waitable', default='false')

    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera'
    )

    threading_type_arg = DeclareLaunchArgument(
        'threading_type',
        default_value='single',
        description='Threading type'
    )

    use_waitable_arg = DeclareLaunchArgument(
        'use_waitable',
        default_value='false',
        description='Use waitable'
    )

    yolo_cmd = Node(
        package='yolo_rt',
        executable='yolo_rt',
        name='yolo',
        output='screen',
        parameters=[{
            'weights_path': weights_path,
            'use_waitable': use_waitable
        }],
        arguments=[threading_type]
    )

    # realsense cmd with ifcondition to enable camera
    realsense_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(realsense_pkd_dir, 'launch', 'rs_launch.py')),
        condition=IfCondition(enable_camera),
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
        }.items()
    )

    # Launch Description
    launch_description = LaunchDescription()

    # launch_description.add_action(enable_camera_arg)
    launch_description.add_action(threading_type_arg)
    launch_description.add_action(use_waitable_arg)
    launch_description.add_action(yolo_cmd)
    # launch_description.add_action(realsense_cmd)

    return launch_description
