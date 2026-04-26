from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('dav2_nav'),
        'config', 'nav_params.yaml')

    return LaunchDescription([
        Node(package='camera_ros', executable='camera_node',
             name='camera_node',
             parameters=[{'orientation': 180, 'width': 640,
                          'height': 480, 'format': 'BGR888'}],
             remappings=[('/camera/image_raw', '/image_raw')]),
        Node(package='dav2_nav', executable='dav2_node',
             name='dav2_node', parameters=[config]),
        Node(package='dav2_nav', executable='nav_node',
             name='nav_node', parameters=[config]),
        Node(package='dav2_nav', executable='controller_node',
             name='controller_node', parameters=[config]),
    ])
