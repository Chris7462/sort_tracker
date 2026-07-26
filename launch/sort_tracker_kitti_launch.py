from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('sort_tracker')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bagfile) clock if true'
    )

    declare_detector_package = DeclareLaunchArgument(
        'detector_package',
        default_value='fcos_object_detection',
        description='Package providing the detector paired with the '
                     'tracker for this demo. Alternative: '
                     'rf_detr_detection. Must publish vision_msgs/'
                     'Detection2DArray and accept input_topic as a launch '
                     'argument.'
    )

    declare_detector_launch_file = DeclareLaunchArgument(
        'detector_launch_file',
        default_value='fcos_object_detection_launch.py',
        description='Launch file, relative to <detector_package>/launch, '
                     'that starts the detector. Alternative: '
                     'rf_detr_detection_launch.py.'
    )

    declare_detection_input_topic = DeclareLaunchArgument(
        'detection_input_topic',
        default_value='fcos_object_detection/detection_array',
        description='Must match the detector\'s actual output topic - '
                     '"fcos_object_detection/detection_array" for fcos, '
                     '"rf_detr_detection" for rf_detr_detection.'
    )

    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare(LaunchConfiguration('detector_package')),
                'launch',
                LaunchConfiguration('detector_launch_file')
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/kitti/camera/color/left/image_raw',
        }.items()
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'sort_tracker_rviz_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': '/kitti/camera/color/left/image_raw',
            'detection_input_topic': LaunchConfiguration('detection_input_topic'),
        }.items()
    )

    bag_exec = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1.0',
             '/data/kitti/raw/2011_09_29_drive_0071_sync_bag',
             '--clock',
             '--qos-profile-overrides-path',
             join(pkg_share, 'config', 'qos_override_offline.yaml')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_detector_package,
        declare_detector_launch_file,
        declare_detection_input_topic,
        detector_launch,
        rviz_launch,
        TimerAction(
            period=3.0,  # delay these nodes for 3.0 seconds.
            actions=[
                bag_exec
            ]
        )
    ])
