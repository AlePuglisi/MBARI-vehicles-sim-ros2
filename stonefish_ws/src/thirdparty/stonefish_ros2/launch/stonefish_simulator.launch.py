from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    simulation_data = LaunchConfiguration('simulation_data')
    scenario_desc = LaunchConfiguration('scenario_desc')
    simulation_rate = LaunchConfiguration('simulation_rate')
    window_res_x = LaunchConfiguration('window_res_x')
    window_res_y = LaunchConfiguration('window_res_y')
    rendering_quality = LaunchConfiguration('rendering_quality')
       
    simulation_data_arg = DeclareLaunchArgument(
        'simulation_data',
        default_value = ''
    )

    scenario_desc_arg = DeclareLaunchArgument(
        'scenario_desc',
        default_value = ''
    )

    simulation_rate_arg = DeclareLaunchArgument(
        'simulation_rate',
        default_value = '100.0'
    )

    window_res_x_arg = DeclareLaunchArgument(
        'window_res_x',
        default_value = '800'
    )

    window_res_y_arg = DeclareLaunchArgument(
        'window_res_y',
        default_value = '600'
    )

    rendering_quality_arg = DeclareLaunchArgument(
        'rendering_quality',
        default_value = 'high'
    )

    stonefish_simulator_node = Node(
            package='stonefish_ros2',
            executable='stonefish_simulator',
            namespace='stonefish_ros2',
            name='stonefish_simulator',
            arguments=[simulation_data, scenario_desc, simulation_rate, window_res_x, window_res_y, rendering_quality],
            output='screen',
            #prefix=['xterm -e gdb -ex run --args']
    )

    return LaunchDescription([
        simulation_data_arg,
        scenario_desc_arg,
        simulation_rate_arg,
        window_res_x_arg,
        window_res_y_arg,
        rendering_quality_arg,
        stonefish_simulator_node
    ])