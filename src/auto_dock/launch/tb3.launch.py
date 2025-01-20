#! /usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory("auto_dock")
    package_dir_nav2 = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time", default=True)

    nav2_map = os.path.join(package_dir, "map", "my_map.yaml")
    nav2_params = os.path.join(package_dir_nav2, "params", "nav2_params.yaml")

    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        )
    )
    
    nav2_bringup = IncludeLaunchDescription(    
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("nav2_bringup"),
                "launch",
                "bringup_launch.py",
            )
        ),
        launch_arguments=[
            ("map", nav2_map),
            ("use_sim_time", use_sim_time),
            ("params_file", nav2_params),
        ],
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                get_package_share_directory('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            ])
        ],
        output='screen'
    )

    return LaunchDescription([
        turtlebot3_world,
        nav2_bringup,
        rviz2_node,
    ])