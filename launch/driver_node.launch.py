from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    return LaunchDescription(
        [
            SetEnvironmentVariable("QT_QPA_PLATFORM", "xcb"),
            DeclareLaunchArgument(
                "encoding",
                default_value="libcaer_cmp",
                description="Event encoding: 'libcaer_cmp' (compressed) or 'dvs' (uncompressed)",
            ),
            DeclareLaunchArgument(
                "camera_model",
                default_value="dvxplorer",
                description="Camera Model: 'dvxplorer' (dvxplorer) or 'dvxmicro' (dvxplorer-micro)",
            ),
            DeclareLaunchArgument(
                "hardware_in_the_loop",
                default_value="true",
                description="Run with real camera hardware",
            ),
            DeclareLaunchArgument(
                "live_viz",
                default_value="false",
                description="Open a native OpenCV event preview window",
            ),
            Node(
                package="dv_processing_driver",
                executable="dv_processing_driver_node",
                name="event_camera_node",
                parameters=[
                    {
                        "publish_period_ms": 10,  # publish one event message every N ms
                        "max_events": 300000,  # warn if one publish period contains more events
                        "every_nth_px": 4,  # 4 effectively halfs resolution
                        "low_pass_freq": 100.0,
                        "encoding": LaunchConfiguration("encoding"),
                        "camera_model": LaunchConfiguration("camera_model"),
                        "hardware_in_the_loop": ParameterValue(
                            LaunchConfiguration("hardware_in_the_loop"), value_type=bool
                        ),
                        "live_viz": ParameterValue(
                            LaunchConfiguration("live_viz"), value_type=bool
                        ),
                    }
                ],
            ),
        ]
    )
