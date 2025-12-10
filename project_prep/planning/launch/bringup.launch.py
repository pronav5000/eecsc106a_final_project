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

    # usb_cam_node = Node(
    #     package='usb_cam',
    #     executable='usb_cam_node',
    #     name='logitech_camera',
    #     output='screen',
    #     parameters=[{
    #         'video_device': camera_device,
    #         'image_width': 640,
    #         'image_height': 480,
    #         'pixel_format': 'mjpeg',
    #         'camera_frame_id': 'camera_link'
    #     }]
    # )

    # ArUco recognition
    aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros2_aruco'),
                'launch',
                'aruco_recognition.launch.py'
            )
        )
    )

    static_base_world = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_world',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'base_link', 'world'],
        output='screen'
    )

    ar_marker_launch_arg = DeclareLaunchArgument(
        'ar_marker',
        default_value='ar_marker_10'
    )
    ar_marker = LaunchConfiguration('ar_marker')

    planning_tf_node = Node(
        package='planning',
        executable='tf',
        name='tf_node',
        parameters=[{'ar_marker': ar_marker,}],
        output='screen'
    )
        
    ball_node = Node(
        package='perception',
        executable='ball_publisher',
        name='ball_publisher',
        output='screen'
    )

    cup_node = Node(
        package='perception',
        executable='cup_publisher',
        name='cup_publisher',
        output='screen'
    )

    ik_node = Node(
        package='planning',
        executable='ik',
        name='ik',
        output='screen'
    )


    taskmaster_node = Node(
        package='planning',
        executable='task_master',
        name='task_master',
        output='screen'
    )

    # taskmaster_node = Node(
    #     package='planning',
    #     executable='main',
    #     name='main',
    #     output='screen'
    # )

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
        ar_marker_launch_arg,
        camera_device_arg,
        # usb_cam_node,
        # static_tf_camera,
        planning_tf_node,
        static_base_world,
        ball_node,
        cup_node,
        aruco_launch,
        moveit_launch,
        ik_node,
        taskmaster_node,
        shutdown_on_exit
    ])
