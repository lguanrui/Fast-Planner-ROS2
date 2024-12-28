from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution, PythonExpression, ThisLaunchFileDir
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ComposableNode
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Declare arguments
    launch_args = [DeclareLaunchArgument('name',            default_value='quadrotor'),
                   DeclareLaunchArgument('platform_type',   default_value='race2'),
                   DeclareLaunchArgument('camera_type',     default_value='d455',       description="Supported cameras: 'd435i' or 'd455' "),  
                   DeclareLaunchArgument('use_nmpc',        default_value='false'),
                   DeclareLaunchArgument('use_microxrce',   default_value='true'),
                   DeclareLaunchArgument('field',           default_value='indoor'),
                   DeclareLaunchArgument('use_ov',          default_value='false'),
    ]

    # Use the LaunchConfiguration for each argument
    name = LaunchConfiguration('name')

    local_planner_config = PathJoinSubstitution([
        get_package_share_directory('plan_manage'),
        TextSubstitution(text='config'), 
        TextSubstitution(text='kino_algorithm.yaml')]
    )

    # Define the kino_algorithm node
    kino_replan_node = ComposableNode(
        package='plan_manage',
        plugin='fast_planner::KinoReplanFSM', 
        namespace=name,
        name='kino_replan_fsm',
        parameters=[local_planner_config],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    bspline_tracker_node = ComposableNode(
        package='plan_manage',
        plugin='fast_planner::BsplineTracker', 
        namespace=name,
        name='bspline_tracker',
        parameters=[local_planner_config],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    kino_replan_container = ComposableNodeContainer(
        name='kino_replan_container',
        namespace=name,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            kino_replan_node,
            bspline_tracker_node
        ],
        output='screen',
    )

    # Launch Description
    ld = LaunchDescription(launch_args)
    ld.add_action(
        GroupAction(
            actions=[kino_replan_container]
        ),
    )

    return ld
