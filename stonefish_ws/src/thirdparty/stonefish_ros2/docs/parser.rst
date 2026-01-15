==================
Defining scenarios
==================

Stonefish XML parser
====================

When using the *Stonefish* library, the simulation scenario can be defined using C++ code or loaded from an XML scenario description file.
The second option should be the choice in most situations, as it allows for quick changes in the scenario, without the need for recompiling the code.
One of the reasons to use the C++ code is when an extension of the library functionality is needed.

The *stonefish_ros2* package uses the XML parser approach due to its purpose of delivering a standard simulator node, that can be used without the need for recompilation. The parser class was extended to support ROS2 functionality on top of the core library functionality. 

.. note::

    The **XML syntax** that is necessary to define a simulation scenario is discussed in detail in the `Stonefish documentation <https://stonefish.readthedocs.io/>`_. Below, only the extensions implemented to support the seamless integration of *Stonefish* and ROS2 are presented.

ROS extensions
==============

The following XML syntax extensions were implemented for the *Stonefish*-ROS2 integration:

1. Parsing params passed from the launch file:

.. code-block:: xml

    <robot name="$(param robot_name)">
    
2. Finding files using names of ROS2 packages.

.. code-block:: xml

    <mesh filename="$(find my_package)/data/robot.obj" scale="1.0"/>

3. Definition of topic names, used to publish data from the simulator and receive requests:

    * :ref:`Robot joint and thruster state <robots>`

    * :ref:`Robot servo and thruster setpoints <robots>`

    * :ref:`Sensor measurements <sensors>`

    * :ref:`Trajectory state for animated bodies <misc>`

    * :ref:`Desired pose and twist for animated bodies (manual trajectory) <misc>`

    * :ref:`Contact information <misc>`.