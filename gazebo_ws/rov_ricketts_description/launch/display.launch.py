from ament_index_python.packages import get_package_share_path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    description_path = get_package_share_directory('rov_ricketts_description')
    
    #model_path = description_path / 'urdf/Ricketts.xacro'
    default_rviz_config_path = description_path + '/config/rov_monitor.rviz'
    

    # pkg_dir = get_package_share_directory("rov_ricketts_description")
    xacro_path = os.path.join(description_path, "urdf", "Ricketts.urdf.xacro")

    gui_arg = DeclareLaunchArgument(
        name='gui', default_value='true', choices=['true', 'false'],
        description='Flag to enable joint_state_publisher_gui')
    
    # model_arg = DeclareLaunchArgument(
    #     name='model', default_value=str(
    #         model_path),
    #     description='Absolute path to robot urdf file')
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config', default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')
    
    default_load_arm = 'false'
    load_arm_arg = DeclareLaunchArgument('load_arm',
                                      default_value = default_load_arm)

    # robot_description = ParameterValue(
    #     Command(['xacro ', LaunchConfiguration('model')]),
    #     value_type=str)
    
    robot_description_command = Command(['xacro ', xacro_path, ' load_arm:=', LaunchConfiguration('load_arm')])

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_command}]
    )

    # Depending on gui parameter,
    # either launch joint_state_publisher or joint_state_publisher_gui
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    return LaunchDescription([
        gui_arg,
        #model_arg,
        load_arm_arg,
        rviz_arg,

        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        robot_state_publisher_node,
        rviz_node
    ])
