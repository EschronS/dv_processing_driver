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
                        "slice_period_ms": 5,  # override default 5ms
                        "max_events": 50000,  # override default 50000
                        "every_nth_px": 2,  # override default 2
                    }
                ],
            )
        ]
    )
