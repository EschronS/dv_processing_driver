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
                        "max_msg_duration": 1000000,  # [ns]
                        "max_events": 20000,  # max num of events per msg
                        "every_nth_px": 1,  # override default 2, 4 effectively halfs resolution
                    }
                ],
            )
        ]
    )
