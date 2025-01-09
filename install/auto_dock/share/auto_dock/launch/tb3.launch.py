from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    turtlebot3_gazebo_launch_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),'launch','turtlebot3_world.launch.py')
    nav2_navigation_launch_file = os.path.join(get_package_share_directory('turtlebot3_navigation2'),'launch','navigation2.launch.py' )
    
    description_dir = get_package_share_directory('auto_dock')
    map_file = os.path.join(description_dir, 'map', 'my_map.yaml')

    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=map_file,
            description='/home/tayya/Nav2_Dock_Robot/src/auto_dock/map/my_map.yaml'
        ),

        # TurtleBot3 Gazebo simülasyonu başlatma
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(turtlebot3_gazebo_launch_file),
            launch_arguments={'use_sim_time': 'true'}.items()
        ),

        # Nav2 navigasyon sistemi başlatma
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_navigation_launch_file),
            launch_arguments={
                'map': map_yaml_file,
                'use_sim_time': use_sim_time
            }.items()
        ),
    ])
