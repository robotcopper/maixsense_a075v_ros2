"""Start and activate the MaixSense A075V lifecycle driver."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.substitutions import FindPackageShare
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    camera = LifecycleNode(
        package='maixsense_a075v',
        executable='tof_camera_node',
        name='maixsense_a075v',
        namespace=LaunchConfiguration('namespace'),
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
    )

    configure = RegisterEventHandler(
        OnProcessStart(
            target_action=camera,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(camera),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                )
            ],
        )
    )

    activate = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=camera,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(camera),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'namespace',
                default_value='',
                description='Namespace for the camera topics.',
            ),
            DeclareLaunchArgument(
                'params_file',
                default_value=PathJoinSubstitution(
                    [
                        FindPackageShare('maixsense_a075v'),
                        'config',
                        'tof_camera.yaml',
                    ]
                ),
                description='Camera parameter file.',
            ),
            camera,
            configure,
            activate,
        ]
    )
