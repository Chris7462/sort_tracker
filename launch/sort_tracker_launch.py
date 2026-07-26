from os.path import join

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('sort_tracker')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',  # or 'true' if appropriate
        description='Use simulation time'
    )

    declare_input_topic = DeclareLaunchArgument(
        'input_topic',
        description='Input image topic name. Required - no default, since '
                     'it differs per data source '
                     '(e.g. /kitti/camera/color/left/image_raw or '
                     '/carla/hero/cam2/image). Only consumed by '
                     'sort_tracker_viz_node (for drawing the overlay) - '
                     'sort_tracker_node itself tracks detections and never '
                     'touches images directly. sort_tracker_viz_node will '
                     'refuse to start if this is not provided.'
    )

    declare_detection_input_topic = DeclareLaunchArgument(
        'detection_input_topic',
        default_value='fcos_object_detection/detection_array',
        description='Topic sort_tracker_node subscribes to for detections '
                     '(vision_msgs/Detection2DArray). Must match whatever '
                     'detector is actually running - e.g. '
                     '"fcos_object_detection/detection_array" for '
                     'fcos_object_detection, or "rf_detr_detection" for '
                     'rf_detr_detection (see that package\'s output_topic '
                     'param). Passing the wrong value fails silently: the '
                     'tracker just never produces tracks.'
    )

    params = join(pkg_share, 'param', 'sort_tracker.yaml')
    viz_params = join(pkg_share, 'param', 'sort_tracker_viz.yaml')

    sort_tracker_node = Node(
        package='sort_tracker',
        executable='sort_tracker_node',
        name='sort_tracker_node',
        output='screen',
        parameters=[
            params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'detection_input_topic': LaunchConfiguration('detection_input_topic'),
            }
        ]
    )

    sort_tracker_viz_node = Node(
        package='sort_tracker',
        executable='sort_tracker_viz_node',
        name='sort_tracker_viz_node',
        output='screen',
        parameters=[
            viz_params,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'image_input_topic': LaunchConfiguration('input_topic'),
            }
        ]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_input_topic,
        declare_detection_input_topic,
        sort_tracker_node,
        sort_tracker_viz_node
    ])
