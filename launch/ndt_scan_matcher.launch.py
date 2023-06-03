from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Declare the launch arguments
    pointcloud_map_path_arg = DeclareLaunchArgument(
        'pointcloud_map_path',
        default_value='/home/porizou/autoware_map/sample-map-rosbag/pointcloud_map.pcd',
        description='Path to the pointcloud map')

    return LaunchDescription([
        # Launch arguments
        pointcloud_map_path_arg,
        Node(
            package='livox_ros_driver2',
            executable='msg_MID360',
            output='screen',
        ),
        Node(
            package='autoware_auto_algorithm',
            executable='ndt_scan_matcher',
            output='screen',
        ),
        Node(
            package='map_loader',
            executable='pointcloud_map_loader',
            output='screen',
            parameters=[{'pointcloud_map_path': LaunchConfiguration('pointcloud_map_path')}]
        ),
    ])
