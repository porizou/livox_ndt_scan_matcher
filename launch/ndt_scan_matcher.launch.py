from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare the launch arguments
    pointcloud_map_path_arg = DeclareLaunchArgument(
        'pointcloud_map_path',
        default_value='/home/porizou/autoware_map/sample-map-rosbag/pointcloud_map.pcd',
        description='Path to the pointcloud map'
    )

    livox_ros_driver2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [FindPackageShare('livox_ros_driver2'), '/launch/rviz_MID360_launch.py']
        )
    )

    ndt_scan_matcher_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [FindPackageShare('ndt_scan_matcher'), '/launch/ndt_scan_matcher.launch.xml']
        )
    )

    map_loader_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            [FindPackageShare('map_loader'), '/launch/pointcloud_map_loader.launch.xml']
        ),
        launch_arguments={'pointcloud_map_path': LaunchConfiguration('pointcloud_map_path')}.items()
    )

    return LaunchDescription([
        # Launch arguments
        pointcloud_map_path_arg,
        # Launch configurations
        livox_ros_driver2_launch,
        ndt_scan_matcher_launch,
        map_loader_launch
    ])
