from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dv_processing_driver",
                executable="dv_processing_driver_node",
                name="event_camera_node",
                parameters=[
                    {
                        "publish_period_ms": 10,  # publish one event message every N ms
                        "max_events": 30000,  # warn if one publish period contains more events
                        "every_nth_px": 1,  # override default 2, 4 effectively halfs resolution
                        "low_pass_freq": 50.0,
                    }
                ],
            )
        ]
    )
