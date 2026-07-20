from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sort_tracker')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (bagfile) clock if true'
    )

    bag_exec = ExecuteProcess(
        cmd=['ros2', 'bag', 'play', '-r', '1.0',
             '/data/kitti/raw/2011_09_29_drive_0071_sync_bag',
             '--clock',
             '--qos-profile-overrides-path',
             join(pkg_share, 'config', 'qos_override_offline.yaml')]
    )

    fcos_object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(get_package_share_directory('fcos_object_detection'),
                 'launch', 'fcos_object_detection_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': 'kitti/camera/color/left/image_raw',
        }.items()
    )

    sort_tracker_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(pkg_share, 'launch', 'sort_tracker_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    viz_params = join(pkg_share, 'param', 'sort_tracker_viz.yaml')

    sort_tracker_viz_node = Node(
        package='sort_tracker',
        executable='sort_tracker_viz_node',
        name='sort_tracker_viz_node',
        output='screen',
        parameters=[
            viz_params,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', join(pkg_share, 'rviz', 'sort_tracker.rviz')]
    )

    return LaunchDescription([
        declare_use_sim_time,
        fcos_object_detection_launch,
        sort_tracker_launch,
        sort_tracker_viz_node,
        rviz_node,
        TimerAction(
            period=3.0,  # delay these nodes for 3.0 seconds.
            actions=[
                bag_exec
            ]
        )
    ])
