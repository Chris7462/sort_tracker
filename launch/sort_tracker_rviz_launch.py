from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sort_tracker')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bagfile/CARLA) clock if true'
    )

    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        description='Input image topic name. Required - no default. '
                     'Passed through to sort_tracker_launch.py. Note: this '
                     'launch file does not start a detector - the caller '
                     'must launch one that publishes to '
                     'detection_input_topic or sort_tracker_node will '
                     'never produce any tracks.'
    )

    declare_detection_input_topic = DeclareLaunchArgument(
        'detection_input_topic',
        default_value='fcos_object_detection/detection_array',
        description='Topic sort_tracker_node subscribes to for detections. '
                     'Must match the detector actually running - see '
                     'sort_tracker_launch.py for details. Passed through.'
    )

    sort_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'sort_tracker_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('input_topic'),
            'detection_input_topic': LaunchConfiguration('detection_input_topic'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(pkg_share, 'rviz', 'sort_tracker.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_input_topic,
        declare_detection_input_topic,
        sort_tracker_launch,
        rviz_node,
    ])
