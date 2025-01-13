import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription ,DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare 
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    default_map_path = os.path.join(get_package_share_directory('auto_dock'),'map','my_map.yaml')

    DeclareLaunchArgument('map',default_value=default_map_path, description='Full path to map file to load'),
    
    turtlebot3_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        )
    )
    
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),'launch','bringup_launch.py'])
        ),
        
        launch_arguments={
            'use_sim_time': 'True',
            'autostart': 'True',
            'map': LaunchConfiguration('map')
        }.items()
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=[
            '-d',
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'rviz',
                'nav2_default_view.rviz'
            ])
        ],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument('map',default_value=default_map_path, description='Full path to map file to load'),
        turtlebot3_world,
        nav2_bringup,
        rviz2_node,
    ])