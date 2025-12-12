from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="dv_processing_driver",
                executable="dv_processing_driver_node",
                name="event_camera_node",
                parameters=[
                    {
                        "slice_period_ms": 10,  # override default 5ms
                        "max_events": 100000,  # override default 50000
                        "every_nth_px": 4,  # override default 2
                    }
                ],
            )
        ]
    )
