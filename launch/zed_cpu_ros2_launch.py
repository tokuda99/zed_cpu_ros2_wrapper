from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    node = Node(
        package="zed_cpu_ros2_wrapper",
        executable="zed_cpu_ros2_wrapper_node",
        name="zed_cpu_ros2_wrapper_node",
        output="screen",
        parameters=[
            {
                "resolution": LaunchConfiguration("resolution"),
                "fps": LaunchConfiguration("fps"),
                "publish_rate": LaunchConfiguration("publish_rate"),
                "use_camera_buffer_timestamps": LaunchConfiguration(
                    "use_camera_buffer_timestamps"
                ),
                "intra_process_comm" : LaunchConfiguration("intra_process_comm"),
                "left_image_topic_name": LaunchConfiguration("left_image_topic_name"),
                "right_image_topic_name": LaunchConfiguration("right_image_topic_name"),
                "left_camera_frame_id": LaunchConfiguration("left_camera_frame_id"),
                "right_camera_frame_id": LaunchConfiguration("right_camera_frame_id"),
                "use_sensor_data_qos": LaunchConfiguration("use_sensor_data_qos"),
            }
        ],
    )

    def add_launch_arg(name: str, default_value):
        return DeclareLaunchArgument(name, default_value=default_value)

    return LaunchDescription(
        [
            add_launch_arg("resolution", "HD1080"),
            add_launch_arg("fps", "15"),
            add_launch_arg("publish_rate", "10.0"),
            add_launch_arg("use_camera_buffer_timestamps", "false"),
            add_launch_arg("intra_process_comm", "false"),
            add_launch_arg("left_image_topic_name", "zed/left_camera/image_raw"),
            add_launch_arg("right_image_topic_name", "zed/right_camera/image_raw"),
            add_launch_arg("left_camera_frame_id", "left_camera"),
            add_launch_arg("right_camera_frame_id", "right_camera"),
            add_launch_arg("use_sensor_data_qos", "false"),
            node,
        ]
    )
