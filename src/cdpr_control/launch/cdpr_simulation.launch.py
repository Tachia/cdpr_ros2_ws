from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package share directory
    pkg_path = get_package_share_directory('cdpr_control')
    
    return LaunchDescription([
        # Launch Gazebo with the CDPR world
        ExecuteProcess(
            cmd=['gz', 'sim', '-r', os.path.join(pkg_path, 'worlds', 'cdpr_8cables_valid.sdf')],
            output='screen',
            shell=True
        ),
        
        # CDPR Controller Node
        Node(
            package='cdpr_control',
            executable='cdpr_controller',
            name='cdpr_controller',
            output='screen'
        ),
        
        # Static TF broadcaster for world frame
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'world', 'map'],
            output='screen'
        ),
    ])
