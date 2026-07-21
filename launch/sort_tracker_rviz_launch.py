from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sort_tracker')
    fcos_pkg_share = get_package_share_directory('fcos_object_detection')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (bagfile/CARLA) clock if true'
    )

    declare_image_input_topic = DeclareLaunchArgument(
        'image_input_topic',
        description='Input image topic name. Required - no default, since '
                     'it differs per data source '
                     '(e.g. /kitti/camera/color/left/image_raw or '
                     '/carla/hero/cam2/image). Passed through to both the '
                     'fcos_object_detection node and sort_tracker_viz_node, '
                     'each of which will refuse to start if this is not '
                     'provided.'
    )

    fcos_object_detection_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            join(fcos_pkg_share, 'launch', 'fcos_object_detection_launch.py')
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'input_topic': LaunchConfiguration('image_input_topic'),
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
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'image_input_topic': LaunchConfiguration('image_input_topic'),
            }
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
        declare_image_input_topic,
        fcos_object_detection_launch,
        sort_tracker_launch,
        sort_tracker_viz_node,
        rviz_node,
    ])
