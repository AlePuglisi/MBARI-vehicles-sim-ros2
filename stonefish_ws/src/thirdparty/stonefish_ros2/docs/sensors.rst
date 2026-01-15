.. _sensors:

=======
Sensors
=======

The *Stonefish* library implements a wide range of sensor simulations. Each of the sensors produces a specific output, that can be retrieved. The *stonefish_ros2* package implements automatic allocation of publishers and generation of appropriate messages, published when the new sensor data is available. The only definition that the user has to add to the standard definiton of a sensor, to use this functionality, is the name of the topic used by the publisher:

.. code-block:: xml

    <sensor name="..." type="...">
        <!-- standard sensor definitions -->
        <ros_publisher topic="/topic_name"/>
    </sensor>

The appropriate message or group of messages is published for each of the sensors. For some of them there is a secondary optional topic that can be specified after the standard topic, using an attribute name given below.

Moreover, an additional functionality is available - an automatically generated service to enable and disable the sensor, for example, to get better performance when all sensors are not needed at the same time. A topic used by the service has to be given as follows:

.. code-block:: xml

    <sensor name="..." type="...">
        <!-- standard sensor definitions -->
        <!-- ros_publisher definition -->
        <ros_service set_enabled="/service_name"/>
    </sensor>

The above service is of type ``std_srvs::srv::SetBool``.

Joint sensors
=============

Rotary encoder
--------------

Message type: ``sensor_msgs::msg::JointState``

Force-torque (6-axis)
---------------------

Message type: ``geometry_msgs::msg::WrenchStamped``

Link sensors
============

Accelerometer
-------------

Message type: ``geometry_msgs::msg::AccelWithCovarianceStamped``

Gyroscope
---------

Message type: ``geometry_msgs::msg::TwistWithCovarianceStamped``

IMU
---

Message type: ``sensor_msgs::msg::Imu``

Odometry
--------

Message type: ``nav_msgs::msg::Odometry``

GPS
---

Message type: ``sensor_msgs::msg::NavSatFix``

Doppler velocity log (DVL)
--------------------------

Message type: ``stonefish_ros2::msg::DVL``

Second message type (*altitude_topic*): ``sensor_msgs::msg::Range``

Pressure
--------

Message type: ``sensor_msgs::msg::FluidPressure``

INS
---

Message type: ``stonefish_ros2::msg::INS``

Second message type (*odometry_topic*): ``nav_msgs::msg::Odometry``

Multi-beam
----------

Message type: ``sensor_msgs::msg::LaserScan``

Second message type (*pcl_topic*): ``sensor_msgs::msg::PointCloud2``

Profiler
--------

Message type: ``sensor_msgs::msg::LaserScan``

Vision sensors
==============

Color camera
------------

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::CameraInfo``

Depth camera
------------

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::CameraInfo``

Optical flow camera
-------------------

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::CameraInfo``

Third message type: ``sensor_msgs::msg::Image``

Thermal camera
--------------

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::CameraInfo``

Third message type: ``sensor_msgs::msg::Image``

Segmentation camera
-------------------

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::CameraInfo``

Third message type: ``sensor_msgs::msg::Image``

Event-based camera
------------------

Message type: ``stonefish_ros2::msg::EventArray``

Forward-looking sonar (FLS)
---------------------------

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::Image``

Mechanical scanning imaging sonar (MSIS)
----------------------------------------

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::Image``

Side-scan sonar (SSS)
---------------------

Message type: ``sensor_msgs::msg::Image``

Second message type: ``sensor_msgs::msg::Image``

