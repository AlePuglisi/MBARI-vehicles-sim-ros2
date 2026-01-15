.. _robots:

======
Robots
======

The definition of robots is one of the key features of the *Stonefish* library. Each robot is constructed from dynamic bodies, connected through joints (kinematic tree). Then it is equipped with sensors and actuators. All details on creating robots using the XML syntax are explained in the *Stonefish* documentation. Only parser extensions implemented for the integration of *Stonefish* and ROS2 are described here.

When creating a robot in the simulation scenario, it is typical to attach various actuators to its joints and links.
Most commonly used actuators for underwater robots include servos and thrusters. A simple extension of the XML syntax, presented below, was developed to deal with both of these actuator types. Other types of actuators and all types of sensors and communication devices are treated separately (see following chapters of the documentation).

.. _joint-state:

Joint state
===========

The joint state of the robot can be retrieved in two ways, by reading it from the servo actuators or from the attached rotary encoders. The first option works for all actively driven joints, as the user has to define servos for these joints, to be able to move them. The second option requires separate definition of an encoder for each of the joints. The standard approach is the first one and the second one can be used in case of robots with passive links. The *stonefish_ros2* package greatly simplifies reading the joint state of the whole robot, by introducing a single definition, enabling the standard joint state publisher, automatically gathering data from all defined servo actuators:

.. code-block:: xml

    <robot name="..." fixed="...">
        <!-- all standard robot definitions -->
        <ros_publisher servos="/my_joint_state"/>
    </robot>

The joint state is published using the standard ROS2 message of type ``sensor_msgs::msg::JointState``. With the number of elements in the vectors equal to the number of servo actuators attached to the robot. The names of the joints are filled based on the defined robot name and joint names, e.g., ``robot_name/joint1_name``.

Servo setpoints
===============

Equally important as retrieving the joint state, is updating the actuators' setpoints. Similarly to the mechanism described above, a single definition is needed to enable the standard servo setpoint subscriber, which automatically applies new setpoints to all of the robot's servo actuators:

.. code-block:: xml

    <robot name="..." fixed="...">
        <!-- all standard robot definitions -->
        <ros_subscriber servos="/my_joint_setpoints"/>
    </robot>

The setpoints are expected to be published using the standard ROS2 message type ``sensor_msgs::msg::JointState``. The *stonefish_ros2* package identifies the servo actuators, to which the setpoints should be applied to, by their names (see :ref:`joint-state`). Moreover, **two control modes** are implemented: **position** and **velocity**. The mode is automatically detected by checking the size of the position and velocity vectors, defined in the message, i.e., if the user wants to use position mode, only the position vector should be filled.

Thrusters
=========

Similarly to the servo actuators the *stonefish_ros2* package automatically aggregates information for all thrusters defined within a robot. A single definition is needed to enable the standard thruster setpoint subscriber:

.. code-block:: xml

    <robot name="..." fixed="...">
        <!-- all standard robot definitions -->
        <ros_subscriber thrusters="/my_thruster_setpoints"/>
    </robot>

The setpoints are expected to be published using a message of type ``std_msgs::msg::Float64MultiArray``, containing a vector of values with a size corresponding to the number of defined thrusters. Each of the **setpoints**, proportional to the desired rotational velocity of the propeller, is expected to be **between -1 and 1**. The value of 0 naturally corresponds to a stopped propeller, while the value of 1 corresponds to the maximum rotational velocity, specified during the actuator definition. The sign of the setpoint corresponds to the propeller's rotation direction. Additionally, *stonefish_ros2* package implements automatic publishing of thruster state, using a custom message type ``stonefish_ros2/ThrusterState``. The contents of the message include the current thruster setpoints, the rotational velocities of the propellers, and the generated thrusts and torques. Enabling the thruster state publisher requires a following definition:

.. code-block:: xml

    <robot name="..." fixed="...">
        <!-- all standard robot definitions -->
        <ros_publisher thrusters="/my_thruster_state"/>
    </robot>

.. warning::

    When a robot is equipped both with servos and thrusters, the topic definitions have to be aggregated as attributes of the same ``ros_publisher`` and ``ros_subscriber`` tags.