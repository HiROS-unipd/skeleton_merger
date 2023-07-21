from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    node = Node(
        package='hiros_skeleton_merger',
        executable='hiros_skeleton_merger',
        name='skeleton_merger',
        namespace='hiros',
        output='screen',
        parameters=[
            {'input_topic': '/input/topic'},
            {'output_topic': '/output/topic'},
            {'n_detectors': -1},
            {'max_delta_t': -1.},
            {'max_position_delta': -1.},
            {'max_orientation_delta': -1.},
            {'pelvis_marker_id': -1},
        ]
    )

    ld.add_action(node)
    return ld
