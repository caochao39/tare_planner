from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    rosbag_record_arg = DeclareLaunchArgument(
        'rosbag_record',
        default_value='false',
        description='Record rosbag')

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value='Desktop',
        description='Path to save bag')

    bag_name_prefix_arg = DeclareLaunchArgument(
        'bag_name_prefix',
        default_value='tare_campus',
        description='Prefix for the bag name')

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz')

    use_boundary_arg = DeclareLaunchArgument(
        'use_boundary',
        default_value='false',
        description='Use boundary for navigation')

    explore_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [get_package_share_directory('tare_planner'), '/explore.launch.py']),
        launch_arguments={
            'scenario': 'campus',
            'rosbag_record': LaunchConfiguration('rosbag_record'),
            'bag_path': LaunchConfiguration('bag_path'),
            'bag_name_prefix': LaunchConfiguration('bag_name_prefix'),
            'rviz': LaunchConfiguration('rviz')
        }.items())

    navigation_boundary_node = Node(
        package='tare_planner',
        executable='navigationBoundary',
        name='navigationBoundary',
        output='screen',
        parameters=[
            {'boundary_file_dir': get_package_share_directory(
                'tare_planner') + '/data/boundary.ply'},
            {'sendBoundary': True},
            {'sendBoundaryInterval': 2}
        ],
        condition=IfCondition(LaunchConfiguration('use_boundary')))

    return LaunchDescription([
        rosbag_record_arg,
        bag_path_arg,
        bag_name_prefix_arg,
        rviz_arg,
        use_boundary_arg,
        explore_launch,
        navigation_boundary_node,
    ])
