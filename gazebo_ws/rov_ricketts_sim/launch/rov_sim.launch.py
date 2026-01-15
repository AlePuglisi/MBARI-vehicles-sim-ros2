import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command

from launch_ros.actions import Node



def generate_launch_description():

    ## useful packages shared path
    sim_pkg_name = 'rov_ricketts_sim'
    description_pkg_name = 'rov_ricketts_description'

    sim_pkg_path = get_package_share_directory(sim_pkg_name)
    description_pkg_path = get_package_share_directory(description_pkg_name)

    ## launch arguments (to pass by command line)
    # chose if load or not robot arm
    default_load_arm = 'false'
    load_arm_arg = DeclareLaunchArgument('load_arm',
                                      default_value = default_load_arm)
    # chose rviz configuration file  
    default_rviz_config_path = description_pkg_path + '/config/rov_monitor.rviz'
    rviz_arg = DeclareLaunchArgument(
        name='rviz_config', default_value=str(default_rviz_config_path),
        description='Absolute path to rviz config file')
    
    ## launch configuration (to pass to other ROS2 launch)
    # path to empty underwater gazebo sdf world  
    use_sim_time = LaunchConfiguration('use_sim_time')
    sim_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        )

    default_world = os.path.join(
        sim_pkg_path,
        'worlds',
        'underwater_world.sdf'
        ) 
    world = LaunchConfiguration('world')
    # chose simulation world path 
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world,
        description='World to load'
        )

    gz_model_env =  SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
             sim_pkg_path + '/models/')

    
    ## Process URDF file
    xacro_path = os.path.join(description_pkg_path,'urdf','Ricketts.urdf.xacro')
    # robot_description_config = xacro.process_file(xacro_file).toxml()
    robot_description_command = Command(['xacro ', xacro_path, ' load_arm:=', LaunchConfiguration('load_arm')])
    
    # Create a robot_state_publisher node
    params = {'robot_description': robot_description_command, 'use_sim_time': use_sim_time}

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # joint_state_publisher_node = Node(
    #             package="joint_state_publisher",
    #             executable="joint_state_publisher",
    #             name="joint_state_publisher",
    # )

    ## Launch Rviz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
    )

    ## Launch Gazebo related tools
    # Include the Gazebo launch file, provided by the ros_gz_sim package
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                    launch_arguments={'gz_args': ['-r -v4 ', world], 'on_exit_shutdown': 'true'}.items()
             )

    # Run the spawner node from the ros_gz_sim package. The entity name doesn't really matter if you only have a single robot.
    spawn_entity = Node(package='ros_gz_sim', executable='create',
                        arguments=['-topic', 'robot_description',
                                   '-name', 'rov_ricketts', 
                                   '-x', '-7.0',
                                   '-y', '10.0',
                                   '-z', '0.8',
                                   '-Y', '-0.0'],#'3.14'],
                        output='screen')
    
    # Create ROS-GZ bridge from config file
    bridge_params = os.path.join(sim_pkg_path,'config','gz_bridge.yaml')
    ros_gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ]
    )

    # ros_gz_image_bridge = Node(
    #     package="ros_gz_image",
    #     executable="image_bridge",
    #     arguments=["/camera/image_raw"]
    # )

    # Launch all:
    return LaunchDescription([
        world_arg,
        sim_arg,
        load_arm_arg, 
        rviz_arg,

        robot_state_publisher_node,
        #joint_state_publisher_node,
        rviz_node,

        gz_model_env,
        gazebo,
        spawn_entity,
        ros_gz_bridge,
        # ros_gz_image_bridge
    ])