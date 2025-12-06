from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    camera_device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video0'
    )
    camera_device = LaunchConfiguration('camera_device')

    usb_cam_node = Node(
        package='usb_cam',
        executable='usb_cam_node',
        name='logitech_camera',
        output='screen',
        parameters=[{
            'video_device': camera_device,
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'mjpeg',
            'camera_frame_id': 'camera_link'
        }]
    )

    static_tf_camera = Node(
        package='your_pkg',
        executable='camera_to_base_tf.py',
        name='camera_to_base_tf',
        output='screen'
    )

    ball_node = Node(
        package='your_pkg',
        executable='ball_publisher.py',
        name='ball_publisher',
        output='screen'
    )

    cup_node = Node(
        package='your_pkg',
        executable='cup_publisher.py',
        name='cup_publisher',
        output='screen'
    )

    ik_node = Node(
        package='planning',
        executable='ik',
        name='ik',
        output='screen'
    )

    ur_type = LaunchConfiguration("ur_type", default="ur7e")
    launch_rviz = LaunchConfiguration("launch_rviz", default="true")

    moveit_launch_file = os.path.join(
        get_package_share_directory("ur_moveit_config"),
        "launch",
        "ur_moveit.launch.py"
    )

    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(moveit_launch_file),
        launch_arguments={
            "ur_type": ur_type,
            "launch_rviz": launch_rviz
        }.items()
    )

    shutdown_on_exit = RegisterEventHandler(
        OnProcessExit(
            on_exit=[EmitEvent(event=Shutdown(reason="A node crashed"))]
        )
    )

    return LaunchDescription([
        camera_device_arg,
        usb_cam_node,
        static_tf_camera,
        ball_node,
        cup_node,
        moveit_launch,
        ik_node,
        shutdown_on_exit
    ])
