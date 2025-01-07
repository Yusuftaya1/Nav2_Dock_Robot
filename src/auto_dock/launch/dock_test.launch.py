import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory 

def generate_launch_description():
    nova_carter_dock_params_dir = os.path.join(get_package_share_directory('auto_dock'), 'params')
    params_file = os.path.join(nova_carter_dock_params_dir, 'dock_server.yaml')

    docking_server = Node(
        package='opennav_docking',
        executable='opennav_docking',
        name='docking_server',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([
        docking_server,
    ])