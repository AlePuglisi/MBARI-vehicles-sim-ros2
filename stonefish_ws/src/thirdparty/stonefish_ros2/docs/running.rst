=======
Running
=======

Standard graphical simulator
============================

To run the standard simulator you have to execute the ``stonefish_simulator`` node, passing the required command-line arguments:

.. code-block:: console

    $ ros2 run stonefish_ros2 stonefish_simulator <data_path> <description_path> <rate> <window_res_x> <window_res_y> <rendering_quality>

where:

* data_path - the path to the simulation data directory
* description_path - the path to the scenario description file
* rate - the sampling rate of the simulation [Hz]
* widnow_res - the resolution of the main simulator window [px]
* rendering_quality - the quality of the rendering: low, medium or high.

Another option is to include the provided ``launch/stonefish_simulator.launch.py`` file in your own launch file. It is necessary to override the default arguments. See an example below:

.. code-block:: python

    from launch_ros.substitutions import FindPackageShare
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution

    def generate_launch_description():
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('stonefish_ros2'),
                        'launch',
                        'stonefish_simulator.launch.py'
                    ])
                ]),
                launch_arguments = {
                    'simulation_data' : PathJoinSubstitution([FindPackageShare('my_package'), 'data']),
                    'scenario_desc' : PathJoinSubstitution([FindPackageShare('my_package'), 'scenarios', 'simulation.scn']),
                    'simulation_rate' : '300.0',
                    'window_res_x' : '1200',
                    'window_res_y' : '800',
                    'rendering_quality' : 'high'
                }.items()
            )
        ])

Standard console-based simulator (no GPU)
=========================================

To run the standard simulator without the components utilising the GPU and without the graphical window, you have to execute the ``stonefish_simulator_nogpu`` node, passing the required command-line arguments:  

.. code-block:: console

    $ ros2 run stonefish_ros2 stonefish_simulator_nogpu <data_path> <description_path> <rate>

where:

* data_path - the path to the simulation data directory
* description_path - the path to the scenario description file
* rate - the sampling rate of the simulation [Hz]

Another option is to include the provided ``launch/stonefish_simulator_nogpu.launch.py`` file in your own launch file. It is necessary to override the default arguments. See an example below:

.. code-block:: python

    from launch_ros.substitutions import FindPackageShare
    from launch import LaunchDescription
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from launch.substitutions import PathJoinSubstitution

    def generate_launch_description():
        return LaunchDescription([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('stonefish_ros2'),
                        'launch',
                        'stonefish_simulator_nogpu.launch.py'
                    ])
                ]),
                launch_arguments = {
                    'simulation_data' : PathJoinSubstitution([FindPackageShare('my_package'), 'data']),
                    'scenario_desc' : PathJoinSubstitution([FindPackageShare('my_package'), 'scenarios', 'simulation.scn']),
                    'simulation_rate' : '300.0',
                }.items()
            )
        ])

Implemented services
====================

Some functionality of the simulator node is available through ROS2 services:

* ``enable_currents`` - enable simulation of ocean currents (type *std_srvs::srv::Trigger*)

* ``disable_currents`` - disable simulation of ocean currents (type *std_srvs::srv::Trigger*)