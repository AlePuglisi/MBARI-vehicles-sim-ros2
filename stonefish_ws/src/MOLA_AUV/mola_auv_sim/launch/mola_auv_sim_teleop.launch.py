import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory
import xacro 

def generate_launch_description():
    description_path = get_package_share_directory('mola_auv_sim')
    xacro_path_gt = os.path.join(description_path, "urdf", "mola_auv_gt.urdf.xacro")
    robot_description_gt = xacro.process_file(xacro_path_gt).toxml()

    xacro_path_estim = os.path.join(description_path, "urdf", "mola_auv.urdf.xacro")
    robot_description_estim = xacro.process_file(xacro_path_estim).toxml()

    # Declare launch arguments
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='mola_auv',
        description='Name of the robot'
    )

    # Group action with namespace
    namespace_action = GroupAction(
        actions=[
            # Include the simulator launch file
            IncludeLaunchDescription(
                PathJoinSubstitution([
                    FindPackageShare('stonefish_ros2'), 'launch', 'stonefish_simulator.launch.py'
                ]),
                launch_arguments={
                    'simulation_data': PathJoinSubstitution([
                        FindPackageShare('mola_auv_sim'), 'data'
                    ]),
                    'scenario_desc': PathJoinSubstitution([
                        FindPackageShare('mola_auv_sim'), 'scenarios', 'tank.scn'
                    ]),
                    'simulation_rate': '100.0',
                    'window_res_x': '1200',
                    'window_res_y': '800',
                    'rendering_quality': 'high'
                }.items()
            ),
        ]
    )

    odom2tf_node = Node(
            package='mola_auv_control',
            executable='odom2tf',
            output='screen',
        )
    
    joy_node = Node(
            package='joy',
            executable='joy_node',
            parameters=[{
                'deadzone': 0.1,            # Default is 0.05
                'autorepeat_rate': 5.0,     # Hz, set to 0.0 to disable autorepeat
                'sticky_buttons': False,
                'coalesce_interval_ms': 1,  # Milliseconds between published messages
            }],
            output='screen',
        )  
    
    odom2tf_node = Node(
            package='mola_auv_control',
            executable='odom2tf',
            output='screen',
        )
    
    static_tf_node_gt = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "odom", "mola_auv_base_gt"]
        )  
    
    static_tf_node = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            output="screen" ,
            arguments=["0", "0", "0", "0", "0", "0", "odom", "mola_auv_base"]
        )  
       
    mola_auv_joy_teleop_node = Node(
            package='mola_auv_control',
            executable='mola_auv_joy_teleop',
            output='screen'
        )

    mola_auv_joint_states_node = Node(
            package='mola_auv_control',
            executable='mola_auv_joint_states_simple',
            output='screen'
        )
  
    # Ground truth robot state publisher with namespace
    robot_state_publisher_gt_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_gt',
        namespace='ground_truth',
        parameters=[{'robot_description': robot_description_gt}],
        remappings=[
            ('/ground_truth/robot_description', '/robot_description_gt'),
            ('/ground_truth/joint_states', '/joint_states')
        ]
    )   

    # Estimated robot state publisher with namespace
    robot_state_publisher_estim_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_estim',
        namespace='estimated',
        parameters=[{'robot_description': robot_description_estim}],
        remappings=[
            ('/estimated/robot_description', '/robot_description_estim'),
            ('/estimated/joint_states', '/joint_states')
        ]
    )   

    rviz_node =  Node(
            package='rviz2',
            namespace='',
            executable='rviz2',
            name='rviz2',
            arguments=['-d' + os.path.join(get_package_share_directory('mola_auv_sim'), 'cfg', 'mola_auv_sim_stereo.rviz')]
        )
    
    return LaunchDescription([
        robot_name_arg,
        namespace_action,

        odom2tf_node, 
        joy_node,
        mola_auv_joy_teleop_node,
        static_tf_node_gt,
        static_tf_node,
        rviz_node, 
        mola_auv_joint_states_node,
        TimerAction(
            period=2.0,
            actions=[robot_state_publisher_gt_node],
        ),
        TimerAction(
            period=2.0,
            actions=[robot_state_publisher_estim_node],
        )
    
        ])