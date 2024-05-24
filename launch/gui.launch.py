from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    cam_topic = LaunchConfiguration('cam_topic', default='/flir_camera/image_raw')

    return LaunchDescription([
        DeclareLaunchArgument(
            'cam_topic',
            default_value='/flir_camera/image_raw',
            description='Topic name for the camera image'
        ),
        Node(
            package='custom_guyi',
            executable='custom_guyi',
            name='custom_guyi',
            output='screen',
            parameters=[{'cam_topic': cam_topic}]
        )
    ])
