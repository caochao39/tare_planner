from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def launch_tare_node(context, scenario):
    scenario_str = str(scenario.perform(context))
    tare_planner_node = Node(
        package='tare_planner',
        executable='tare_planner_node',
        name='tare_planner_node',
        output='screen',
        # namespace='sensor_coverage_planner',
        parameters=[get_package_share_directory('tare_planner')+'/' + scenario_str + '.yaml'])
    return [tare_planner_node]


def generate_launch_description():
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    scenario = LaunchConfiguration('scenario')

    declare_scenario = DeclareLaunchArgument(
        'scenario',
        default_value='garage',
        description='description for scenario argument')

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='description for rviz argument')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='tare_planner_ground_rviz',
        arguments=[
            '-d', get_package_share_directory('tare_planner')+'/tare_planner_ground.rviz'],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        declare_use_sim_time_cmd,
        declare_scenario,
        declare_rviz,
        GroupAction([rviz_node]),
        OpaqueFunction(function=launch_tare_node, args=[scenario])
    ])
