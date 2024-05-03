"""Single car omnet module launch file"""

import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    """Return launch description"""

    # Package Directories
    pkg_dir = get_package_share_directory('yolo_rt')
    realsense_pkd_dir = get_package_share_directory('realsense2_camera')

    weights_path = os.path.join(pkg_dir, 'weights', 'yolov7-tiny.pt')

    #  launch configuration
    threading_type = LaunchConfiguration('threading_type', default='single')

    # declare launch argument
    threading_type_arg = DeclareLaunchArgument(
        'threading_type',
        default_value='single',
        description='Threading type'
    )
    
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
                }.items()
    )

    yolo_cmd = Node(
        package='yolo_rt',
        executable='yolo.py',
        name='yolo',
        output='screen',
        parameters=[{
            'weights': weights_path
            }],
        # pass argument "multithreaded" to main
        arguments = [threading_type]

    )

    # {'name': 'camera_name',                  'default': 'camera', 'description': 'camera unique name'},
    # {'name': 'serial_no',                    'default': "''", 'description': 'choose device by serial number'},
    # {'name': 'usb_port_id',                  'default': "''", 'description': 'choose device by usb port id'},
    # {'name': 'device_type',                  'default': "''", 'description': 'choose device by type'},
    # {'name': 'config_file',                  'default': "''", 'description': 'yaml config file'},
    # {'name': 'unite_imu_method',             'default': "0", 'description': '[0-None, 1-copy, 2-linear_interpolation]'},
    # {'name': 'json_file_path',               'default': "''", 'description': 'allows advanced configuration'},
    # {'name': 'log_level',                    'default': 'info', 'description': 'debug log level [DEBUG|INFO|WARN|ERROR|FATAL]'},
    # {'name': 'output',                       'default': 'screen', 'description': 'pipe node output [screen|log]'},
    # {'name': 'depth_module.profile',         'default': '0,0,0', 'description': 'depth module profile'},                           
    # {'name': 'enable_depth',                 'default': 'true', 'description': 'enable depth stream'},
    # {'name': 'rgb_camera.profile',           'default': '0,0,0', 'description': 'color image width'},
    # {'name': 'enable_color',                 'default': 'true', 'description': 'enable color stream'},
    # {'name': 'enable_infra1',                'default': 'false', 'description': 'enable infra1 stream'},
    # {'name': 'enable_infra2',                'default': 'false', 'description': 'enable infra2 stream'},
    # {'name': 'infra_rgb',                    'default': 'false', 'description': 'enable infra2 stream'},
    # {'name': 'tracking_module.profile',      'default': '0,0,0', 'description': 'fisheye width'},
    # {'name': 'enable_fisheye1',              'default': 'true', 'description': 'enable fisheye1 stream'},
    # {'name': 'enable_fisheye2',              'default': 'true', 'description': 'enable fisheye2 stream'},
    # {'name': 'enable_confidence',            'default': 'true', 'description': 'enable depth stream'},
    # {'name': 'gyro_fps',                     'default': '0', 'description': "''"},                           
    # {'name': 'accel_fps',                    'default': '0', 'description': "''"},                           
    # {'name': 'enable_gyro',                  'default': 'false', 'description': "''"},                           
    # {'name': 'enable_accel',                 'default': 'false', 'description': "''"},                           
    # {'name': 'enable_pose',                  'default': 'true', 'description': "''"},                           
    # {'name': 'pose_fps',                     'default': '200', 'description': "''"},                           
    # {'name': 'pointcloud.enable',            'default': 'false', 'description': ''}, 
    # {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
    # {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},
    # {'name': 'enable_sync',                  'default': 'false', 'description': "''"},                           
    # {'name': 'align_depth.enable',           'default': 'false', 'description': "''"},                           
    # {'name': 'colorizer.enable',             'default': 'false', 'description': "''"},
    # {'name': 'clip_distance',                'default': '-2.', 'description': "''"},                           
    # {'name': 'linear_accel_cov',             'default': '0.01', 'description': "''"},                           
    # {'name': 'initial_reset',                'default': 'false', 'description': "''"},                           
    # {'name': 'allow_no_texture_points',      'default': 'false', 'description': "''"},                           
    # {'name': 'ordered_pc',                   'default': 'false', 'description': ''},
    # {'name': 'calib_odom_file',              'default': "''", 'description': "''"},
    # {'name': 'topic_odom_in',                'default': "''", 'description': 'topic for T265 wheel odometry'},
    # {'name': 'tf_publish_rate',              'default': '0.0', 'description': 'Rate of publishing static_tf'},
    # {'name': 'diagnostics_period',           'default': '0.0', 'description': 'Rate of publishing diagnostics. 0=Disabled'},
    # {'name': 'decimation_filter.enable',     'default': 'false', 'description': 'Rate of publishing static_tf'},
    # {'name': 'rosbag_filename',              'default': "''", 'description': 'A realsense bagfile to run from as a device'},
    # {'name': 'depth_module.exposure.1',     'default': '7500', 'description': 'Initial value for hdr_merge filter'},
    # {'name': 'depth_module.gain.1',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
    # {'name': 'depth_module.exposure.2',     'default': '1', 'description': 'Initial value for hdr_merge filter'},
    # {'name': 'depth_module.gain.2',         'default': '16', 'description': 'Initial value for hdr_merge filter'},
    # {'name': 'wait_for_device_timeout',      'default': '-1.', 'description': 'Timeout for waiting for device to connect (Seconds)'},
    # {'name': 'reconnect_timeout',            'default': '6.', 'description': 'Timeout(seconds) between consequtive reconnection attempts'},

    # Launch Description
    launch_description = LaunchDescription()

    # launch_description.add_action(realsense_cmd)
    launch_description.add_action(threading_type_arg)
    launch_description.add_action(yolo_cmd)

    return launch_description
