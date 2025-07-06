from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    optitrack_launch_path = os.path.join(
        '/home/ubuntu/ros2_ws/src/mocap4ros2_optitrack/mocap4r2_optitrack_driver/launch',
        'optitrack2.launch.py'
    )

    return LaunchDescription([
        # Launch the OptiTrack driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(optitrack_launch_path),
        ),

        # Activate the lifecycle node after a delay
        TimerAction(
            period=5.0,
            actions=[
                ExecuteProcess(
                    cmd=[
                        'ros2', 'lifecycle', 'set',
                        '/mocap4r2_optitrack_driver_node', 'activate'
                    ],
                    output='screen'
                )
            ]
        ),

	Node(
	    package='marker_pose_publisher_pkg',
	    executable='marker_pose_publisher',
	    name='marker_pose_publisher',
	    output='screen'
	),

        # Static transform from map (Motive) to world (ROS)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_world_tf',
            arguments=['0.3', '-1.4', '0.6', '0', '0', '0', 'map', 'world'],
            output='screen'
        )


    ])





