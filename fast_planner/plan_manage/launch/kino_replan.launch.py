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
                   DeclareLaunchArgument('platform_type',   default_value='race'),
                   DeclareLaunchArgument('camera_type',     default_value='d455',       description="Supported cameras: 'd435i' or 'd455' "),  
                   DeclareLaunchArgument('use_nmpc',        default_value='false'),
                   DeclareLaunchArgument('use_microxrce',   default_value='true'),
                   DeclareLaunchArgument('field',           default_value='indoor'),
                   DeclareLaunchArgument('use_ov',          default_value='false'),
    ]

    # Use the LaunchConfiguration for each argument
    mav_name = LaunchConfiguration('name')
    platform_type = LaunchConfiguration('platform_type')

    map_frame = [TextSubstitution(text='world')]
    sensor_frame = [TextSubstitution(text='base_link')]
    odom_topic_name = [TextSubstitution(text='/'),mav_name, TextSubstitution(text='/odom')]
    depth_topic_name = [TextSubstitution(text='/'),mav_name, TextSubstitution(text='/camera/depth/image_raw')] # RS Depth Topic
    camera_info_topic_name = [TextSubstitution(text='/'),mav_name, TextSubstitution(text='/camera/camera_info')]

    local_planner_config = PathJoinSubstitution([
        get_package_share_directory('plan_manage'),
        TextSubstitution(text='config'), 
        TextSubstitution(text='kino_algorithm.yaml')]
    )

    #NvBlox config directory paths
    base_dir = get_package_share_directory('arpl_autonomy')
    nvblox_base_config_dir = PathJoinSubstitution([base_dir, 'config',platform_type,'default','perception'])
    #camera = PythonExpression(["'rs435i' if '", camera_type, "' == 'd435i' else 'rs455'"])
    #nvblox_realsense_config_dir = PathJoinSubstitution([nvblox_base_config_dir, camera])

    #NvBlox Config files
    nvblox_base_config = PathJoinSubstitution([nvblox_base_config_dir, 'nvblox_base.yaml'])
    #nvblox_realsense_config = PathJoinSubstitution([nvblox_realsense_config_dir, 'nvblox_realsense.yaml'])

    # Define the kino_algorithm node
    kino_replan_node = ComposableNode(
        package='plan_manage',
        plugin='fast_planner::KinoReplanFSM', 
        namespace=mav_name,
        name='kino_replan_fsm',
        parameters=[local_planner_config],
        extra_arguments=[{'use_intra_process_comms': True}],
        remappings=[("cloud","nvblox_node/static_occupancy")],
                   #("sync_depth_odom",odom_topic_name)],
    )

    bspline_tracker_node = ComposableNode(
        package='plan_manage',
        plugin='fast_planner::BsplineTrackerNode', 
        namespace=mav_name,
        name='bspline_tracker',
        parameters=[local_planner_config],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    #NvBlox Node.
    nvblox_node = ComposableNode(
    	namespace=mav_name,
    	package='nvblox_ros',
    	plugin='nvblox::NvbloxNode',
    	name='nvblox_node',
    	remappings=[
    	('depth/image',depth_topic_name),
    	('depth/camera_info',camera_info_topic_name)],
    	parameters=[
                    nvblox_base_config,
                    {'global_frame': map_frame},
                    {'pose_frame': sensor_frame},
                    {'odom_frame': odom_topic_name},
                    {'use_tf_transforms': True},
                    {'use_topic_transforms': False},
                    {'use_lidar': False},
                    {'use_depth': True},
                    {'slice_visualization_attachment_frame_id': map_frame},
                    {'map_clearing_frame_id': map_frame},
                    {'depth_qos': 'DEFAULT'},
                    {'color_qos': 'DEFAULT'},
                    {'esdf_slice_height': 0.0},
                    {'esdf_2d_min_height': -0.1},
                    {'esdf_2d_max_height': 0.3},
                    {'use_intra_process_comms': True}
                    ],
    	extra_arguments=[{'use_intra_process_comms': True}],
    )

    kino_replan_container = ComposableNodeContainer(
        name='kino_replan_container',
        namespace=mav_name,
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            kino_replan_node,
            bspline_tracker_node,
            nvblox_node
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
