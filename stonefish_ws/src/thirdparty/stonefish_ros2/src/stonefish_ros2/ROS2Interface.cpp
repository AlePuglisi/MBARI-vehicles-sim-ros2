/*    
    This file is a part of stonefish_ros2.

    stonefish_ros is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    stonefish_ros is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/

//
//  ROS2Interface.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 04/10/23.
//  Copyright (c) 2023 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2Interface.h"

#include "geometry_msgs/msg/accel_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/fluid_pressure.hpp"
#include "sensor_msgs/msg/range.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"

#include "stonefish_ros2/msg/dvl.hpp"
#include "stonefish_ros2/msg/ins.hpp"
#include "stonefish_ros2/msg/beacon_info.hpp"
#include "stonefish_ros2/msg/int32_stamped.hpp"
#include "stonefish_ros2/msg/event.hpp"
#include "stonefish_ros2/msg/event_array.hpp"

#include <Stonefish/sensors/Sample.h>
#include <Stonefish/sensors/scalar/Accelerometer.h>
#include <Stonefish/sensors/scalar/Gyroscope.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/INS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/scalar/Profiler.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/ThermalCamera.h>
#include <Stonefish/sensors/vision/OpticalFlowCamera.h>
#include <Stonefish/sensors/vision/SegmentationCamera.h>
#include <Stonefish/sensors/vision/EventBasedCamera.h>
#include <Stonefish/sensors/vision/Multibeam2.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/comms/USBL.h>
#include <Stonefish/entities/AnimatedEntity.h>
#include <Stonefish/core/SimulationApp.h>
#include <Stonefish/core/SimulationManager.h>
#include <Stonefish/core/NED.h>

namespace sf
{

ROS2Interface::ROS2Interface(const std::shared_ptr<rclcpp::Node> nh) : nh_(nh)
{
}

void ROS2Interface::PublishTF(std::unique_ptr<tf2_ros::TransformBroadcaster>& br, const sf::Transform& T, const rclcpp::Time& t, const std::string &frame_id, const std::string &child_frame_id) const
{
    sf::Vector3 o = T.getOrigin();
    sf::Quaternion q = T.getRotation();
    
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = t;
    msg.header.frame_id = frame_id;
    msg.child_frame_id = child_frame_id;
    msg.transform.translation.x = o.x();
    msg.transform.translation.y = o.y();
    msg.transform.translation.z = o.z();
    msg.transform.rotation.x = q.x();
    msg.transform.rotation.y = q.y();
    msg.transform.rotation.z = q.z();
    msg.transform.rotation.w = q.w();
    br->sendTransform(msg);
}

void ROS2Interface::PublishAccelerometer(rclcpp::PublisherBase::SharedPtr pub, Accelerometer* acc) const
{
    Sample s = acc->getLastSample();
    sf::Vector3 accelStdDev = sf::Vector3(acc->getSensorChannelDescription(0).stdDev, 
                                          acc->getSensorChannelDescription(1).stdDev,
                                          acc->getSensorChannelDescription(2).stdDev);
    geometry_msgs::msg::AccelWithCovarianceStamped msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = acc->getName();
    msg.accel.accel.linear.x = s.getValue(0);
    msg.accel.accel.linear.y = s.getValue(1);
    msg.accel.accel.linear.z = s.getValue(2);
    msg.accel.covariance[0] = accelStdDev.getX() * accelStdDev.getX();
    msg.accel.covariance[7] = accelStdDev.getY() * accelStdDev.getY();
    msg.accel.covariance[14] = accelStdDev.getZ() * accelStdDev.getZ();
    std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::AccelWithCovarianceStamped>>(pub)->publish(msg);
}

void ROS2Interface::PublishGyroscope(rclcpp::PublisherBase::SharedPtr pub, Gyroscope* gyro) const
{
    Sample s = gyro->getLastSample();
    sf::Vector3 avelocityStdDev = sf::Vector3(gyro->getSensorChannelDescription(0).stdDev, 
                                      gyro->getSensorChannelDescription(1).stdDev,
                                      gyro->getSensorChannelDescription(2).stdDev);
    geometry_msgs::msg::TwistWithCovarianceStamped msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = gyro->getName();
    msg.twist.twist.angular.x = s.getValue(0);
    msg.twist.twist.angular.y = s.getValue(1);
    msg.twist.twist.angular.z = s.getValue(2);
    msg.twist.covariance[21] = avelocityStdDev.getX() * avelocityStdDev.getX();
    msg.twist.covariance[28] = avelocityStdDev.getY() * avelocityStdDev.getY();
    msg.twist.covariance[35] = avelocityStdDev.getZ() * avelocityStdDev.getZ();
    std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>>(pub)->publish(msg);
}

void ROS2Interface::PublishIMU(rclcpp::PublisherBase::SharedPtr pub, IMU* imu) const
{
    Sample s = imu->getLastSample();
    sf::Vector3 rpy = sf::Vector3(s.getValue(0), s.getValue(1), s.getValue(2));
    sf::Quaternion quat(rpy.z(), rpy.y(), rpy.x());
    sf::Vector3 angleStdDev = sf::Vector3(imu->getSensorChannelDescription(0).stdDev,
                                          imu->getSensorChannelDescription(1).stdDev,
                                          imu->getSensorChannelDescription(2).stdDev);
    sf::Vector3 avelocityStdDev = sf::Vector3(imu->getSensorChannelDescription(3).stdDev,
                                              imu->getSensorChannelDescription(4).stdDev,
                                              imu->getSensorChannelDescription(5).stdDev);
    sf::Vector3 accStdDev = sf::Vector3(imu->getSensorChannelDescription(6).stdDev,
                                        imu->getSensorChannelDescription(7).stdDev,
                                        imu->getSensorChannelDescription(8).stdDev);
    //Variance is sigma^2!
    sensor_msgs::msg::Imu msg;
    msg.header.stamp = nh_->get_clock()->now();    
    msg.header.frame_id = imu->getName();
    msg.orientation.x = quat.x();
    msg.orientation.y = quat.y();
    msg.orientation.z = quat.z();
    msg.orientation.w = quat.w();
    msg.orientation_covariance[0] = angleStdDev.getX() * angleStdDev.getX();
    msg.orientation_covariance[4] = angleStdDev.getY() * angleStdDev.getY();
    msg.orientation_covariance[8] = angleStdDev.getZ() * angleStdDev.getZ();
    msg.angular_velocity.x = s.getValue(3);
    msg.angular_velocity.y = s.getValue(4);
    msg.angular_velocity.z = s.getValue(5);
    msg.angular_velocity_covariance[0] = avelocityStdDev.getX() * avelocityStdDev.getX();
    msg.angular_velocity_covariance[4] = avelocityStdDev.getY() * avelocityStdDev.getY();
    msg.angular_velocity_covariance[8] = avelocityStdDev.getZ() * avelocityStdDev.getZ();
    msg.linear_acceleration.x = s.getValue(6);
    msg.linear_acceleration.y = s.getValue(7);
    msg.linear_acceleration.z = s.getValue(8);
    msg.linear_acceleration_covariance[0] = accStdDev.getX() * accStdDev.getX();
    msg.linear_acceleration_covariance[4] = accStdDev.getY() * accStdDev.getY();
    msg.linear_acceleration_covariance[8] = accStdDev.getZ() * accStdDev.getZ();
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Imu>>(pub)->publish(msg);
}

void ROS2Interface::PublishPressure(rclcpp::PublisherBase::SharedPtr pub, Pressure* press) const
{
    Sample s = press->getLastSample();
    sensor_msgs::msg::FluidPressure msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = press->getName();
    msg.fluid_pressure = s.getValue(0);
    msg.variance = press->getSensorChannelDescription(0).stdDev;
    msg.variance *= msg.variance; //Variance is square of standard deviation
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::FluidPressure>>(pub)->publish(msg);
}

void ROS2Interface::PublishDVL(rclcpp::PublisherBase::SharedPtr pub, DVL* dvl) const
{
    //Get data
    Sample s = dvl->getLastSample();
    unsigned short status = (unsigned short)trunc(s.getValue(7));
    sf::Scalar vVariance = dvl->getSensorChannelDescription(0).stdDev;
    vVariance *= vVariance; //Variance is square of standard deviation
    //Publish DVL message
    stonefish_ros2::msg::DVL msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = dvl->getName();
    msg.velocity.x = s.getValue(0);
    msg.velocity.y = s.getValue(1);
    msg.velocity.z = s.getValue(2);
    msg.velocity_covariance[0] = vVariance;
    msg.velocity_covariance[4] = vVariance;
    msg.velocity_covariance[8] = vVariance;
    msg.altitude = (status == 0 || status == 2) ? s.getValue(3) : -1.0;
    std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::DVL>>(pub)->publish(msg);
}

void ROS2Interface::PublishDVLAltitude(rclcpp::PublisherBase::SharedPtr pub, DVL* dvl) const
{
    //Get data
    Sample s = dvl->getLastSample();
    unsigned short status = (unsigned short)trunc(s.getValue(7));
    sf::Vector3 velMax;
    sf::Scalar altMin, altMax;
    dvl->getRange(velMax, altMin, altMax);
    sf::Scalar beamAngle = dvl->getBeamAngle();
    //Publish range message
    sensor_msgs::msg::Range msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = dvl->getName() + "_altitude";
    msg.radiation_type = msg.ULTRASOUND;
    msg.field_of_view = beamAngle*2;
    msg.min_range = altMin;
    msg.max_range = altMax;
    msg.range = (status == 0 || status == 2) ? s.getValue(3) : -1.0;
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::Range>>(pub)->publish(msg);
}

void ROS2Interface::PublishGPS(rclcpp::PublisherBase::SharedPtr pub, GPS* gps) const
{
    Sample s = gps->getLastSample();

    sensor_msgs::msg::NavSatFix msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = gps->getName();
    msg.status.service = msg.status.SERVICE_GPS;

    if(s.getValue(0) > sf::Scalar(90) && s.getValue(1) > sf::Scalar(180)) //Underwater
    {
        msg.status.status = msg.status.STATUS_NO_FIX;
        msg.latitude = 0.0;
        msg.longitude = 0.0;
        msg.altitude = 0.0;
    }
    else
    {
        msg.status.status = msg.status.STATUS_FIX;
        msg.latitude = s.getValue(0);
        msg.longitude = s.getValue(1);
        msg.altitude = 0.0;
    }

    msg.position_covariance[0] = msg.position_covariance[4] = gps->getNoise() * gps->getNoise();
    msg.position_covariance[8] = 1.0;
    msg.position_covariance_type = msg.COVARIANCE_TYPE_DIAGONAL_KNOWN;
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::NavSatFix>>(pub)->publish(msg);
}

void ROS2Interface::PublishOdometry(rclcpp::PublisherBase::SharedPtr pub, Odometry* odom) const
{
    Sample s = odom->getLastSample();
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = "world_ned";
    msg.child_frame_id = odom->getName();
    msg.pose.pose.position.x = s.getValue(0);
    msg.pose.pose.position.y = s.getValue(1);
    msg.pose.pose.position.z = s.getValue(2);
    msg.twist.twist.linear.x = s.getValue(3);
    msg.twist.twist.linear.y = s.getValue(4);
    msg.twist.twist.linear.z = s.getValue(5);
    msg.pose.pose.orientation.x = s.getValue(6);
    msg.pose.pose.orientation.y = s.getValue(7);
    msg.pose.pose.orientation.z = s.getValue(8);
    msg.pose.pose.orientation.w = s.getValue(9);
    msg.twist.twist.angular.x = s.getValue(10);
    msg.twist.twist.angular.y = s.getValue(11);
    msg.twist.twist.angular.z = s.getValue(12);
    std::static_pointer_cast<rclcpp::Publisher<nav_msgs::msg::Odometry>>(pub)->publish(msg);
}

void ROS2Interface::PublishINS(rclcpp::PublisherBase::SharedPtr pub, INS* ins) const
{
    sf::Scalar lat, lon, h;
    SimulationApp::getApp()->getSimulationManager()->getNED()->Ned2Geodetic(0.0, 0.0, 0.0, lat, lon, h);

    Sample s = ins->getLastSample();
    stonefish_ros2::msg::INS msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = ins->getName();
    msg.latitude = s.getValue(4);
    msg.longitude = s.getValue(5);
    msg.origin_latitude = lat;
    msg.origin_longitude = lon;
    msg.pose.north = s.getValue(0);
    msg.pose.east = s.getValue(1);
    msg.pose.down = s.getValue(2);
    msg.pose.roll = s.getValue(9);
    msg.pose.pitch = s.getValue(10);
    msg.pose.yaw = s.getValue(11);
    msg.altitude = s.getValue(3);
    msg.body_velocity.x = s.getValue(6);
    msg.body_velocity.y = s.getValue(7);
    msg.body_velocity.z = s.getValue(8);
    msg.rpy_rate.x = s.getValue(12);
    msg.rpy_rate.y = s.getValue(13);
    msg.rpy_rate.z = s.getValue(14);
    std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::INS>>(pub)->publish(msg);
}

void ROS2Interface::PublishINSOdometry(rclcpp::PublisherBase::SharedPtr pub, INS* ins) const
{
    Sample s = ins->getLastSample();
    nav_msgs::msg::Odometry msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = "world_ned";
    msg.child_frame_id = ins->getName();
    msg.pose.pose.position.x = s.getValue(0);
    msg.pose.pose.position.y = s.getValue(1);
    msg.pose.pose.position.z = s.getValue(2);
    msg.twist.twist.linear.x = s.getValue(6);
    msg.twist.twist.linear.y = s.getValue(7);
    msg.twist.twist.linear.z = s.getValue(8);
    sf::Quaternion q(s.getValue(11), s.getValue(10), s.getValue(9));
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.angular.x = s.getValue(12);
    msg.twist.twist.angular.y = s.getValue(13);
    msg.twist.twist.angular.z = s.getValue(14);
    std::static_pointer_cast<rclcpp::Publisher<nav_msgs::msg::Odometry>>(pub)->publish(msg);
}

void ROS2Interface::PublishForceTorque(rclcpp::PublisherBase::SharedPtr pub, ForceTorque* ft) const
{
    Sample s = ft->getLastSample();    
    geometry_msgs::msg::WrenchStamped msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = ft->getName();
    msg.wrench.force.x = s.getValue(0);
    msg.wrench.force.y = s.getValue(1);
    msg.wrench.force.z = s.getValue(2);
    msg.wrench.torque.x = s.getValue(3);
    msg.wrench.torque.y = s.getValue(4);
    msg.wrench.torque.z = s.getValue(5);
    std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(pub)->publish(msg);
}

void ROS2Interface::PublishEncoder(rclcpp::PublisherBase::SharedPtr pub, RotaryEncoder* enc) const
{
    Sample s = enc->getLastSample();
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = enc->getName();
    msg.name.resize(1);
    msg.position.resize(1);
    msg.velocity.resize(1);
    msg.name[0] = enc->getJointName();
    msg.position[0] = s.getValue(0);
    msg.velocity[0] = s.getValue(1);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(pub)->publish(msg);
}

void ROS2Interface::PublishMultibeam(rclcpp::PublisherBase::SharedPtr pub, Multibeam* mb) const
{
    Sample sample = mb->getLastSample();
    SensorChannel channel = mb->getSensorChannelDescription(0);
    std::vector<sf::Scalar> distances = sample.getData();

    sf::Scalar angRange = mb->getAngleRange();
    size_t angSteps = distances.size();

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = mb->getName();
    
    msg.angle_min = -angRange/sf::Scalar(2); // start angle of the scan [rad]
    msg.angle_max = angRange/sf::Scalar(2); // end angle of the scan [rad]
    msg.angle_increment = angRange/sf::Scalar(angSteps-1); // angular distance between measurements [rad]
    msg.range_min = channel.rangeMin; // minimum range value [m]
    msg.range_max = channel.rangeMax; // maximum range value [m]
    msg.time_increment = 0.; // time between measurements [seconds] - if your scanner is moving, this will be used in interpolating position of 3d points
    msg.scan_time = 0.; // time between scans [seconds]
    
    msg.ranges.resize(angSteps); // range data [m]
    msg.intensities.resize(angSteps); // used to say if measurement is valid

    for(size_t i = 0; i<angSteps; ++i)
    {
        if(distances[i] < channel.rangeMax)
        {
            msg.ranges[i] = distances[i];
            msg.intensities[i] = 1.0;
        }
        else
        {
            msg.ranges[i] = std::numeric_limits<float>::infinity();
            msg.intensities[i] = 0.0;
        }
    }
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pub)->publish(msg);
}

void ROS2Interface::PublishMultibeamPCL(rclcpp::PublisherBase::SharedPtr pub, Multibeam* mb) const
{
    Sample sample = mb->getLastSample();
    SensorChannel channel = mb->getSensorChannelDescription(0);
    std::vector<sf::Scalar> distances = sample.getData();
    sf::Scalar angRange = mb->getAngleRange();
    size_t angSteps = distances.size();

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = mb->getName();
    cloud->header.seq = sample.getId(); // In this message it does not increase automatically
    cloud->height = cloud->width = 1;
    sf::Scalar angleMin = -angRange / sf::Scalar(2);                  // start angle of the scan [rad]
    sf::Scalar angleIncrement = angRange / sf::Scalar(angSteps - 1);  // angular distance between measurements [rad]

    for(size_t i = 0; i < angSteps; ++i)
        if(distances[i] < channel.rangeMax && distances[i] > channel.rangeMin) // Only publish good points
        {  
            double angle = angleMin + i * angleIncrement;
            pcl::PointXYZ pt;
            pt.y = btSin(angle) * distances[i];
            pt.x = btCos(angle) * distances[i];
            pt.z = 0.0;
            cloud->push_back(pt);
        }

    pcl::PCLPointCloud2 pclMsg;
    pcl::toPCLPointCloud2(*cloud, pclMsg);

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = mb->getName();
    pcl_conversions::fromPCL(pclMsg, msg);
    
    try
    {
        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(pub)->publish(msg);
    }
    catch (std::runtime_error& e)
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Runtime error whle publishing multibeam data: " << e.what());
    }
}

void ROS2Interface::PublishProfiler(rclcpp::PublisherBase::SharedPtr pub, Profiler* prof) const
{
    const std::vector<Sample>* hist = prof->getHistory();
    SensorChannel channel = prof->getSensorChannelDescription(1); // range channel

    sensor_msgs::msg::LaserScan msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = prof->getName();
    
    msg.angle_min = hist->front().getValue(0);
    msg.angle_max = hist->back().getValue(0);
    msg.range_min = channel.rangeMin; // minimum range value [m]
    msg.range_max = channel.rangeMax; // maximum range value [m]
    msg.angle_increment = hist->size() == 1 ? 0.0 : hist->at(1).getValue(0) - hist->at(0).getValue(0);
    msg.time_increment = hist->size() == 1 ? 0.0 : hist->at(1).getTimestamp() - hist->at(0).getTimestamp();
    msg.scan_time = hist->back().getTimestamp() - hist->front().getTimestamp();
    
    if(hist->size() == 1) // RVIZ does not display LaserScan with one range
    {
        msg.ranges.resize(2);
        msg.intensities.resize(2);
        msg.ranges[0] = hist->front().getValue(1);
        msg.intensities[0] = msg.ranges[0] == msg.range_max ? 0.1 : 1.0;
        msg.ranges[1] = msg.ranges[0];
        msg.intensities[1] = msg.intensities[0];
    }
    else
    {
        msg.ranges.resize(hist->size());
        msg.intensities.resize(hist->size());
        for(size_t i=0; i<hist->size(); ++i)
        {
            msg.ranges[i] = hist->at(i).getValue(1);
            msg.intensities[i] = msg.ranges[i] == msg.range_max ? 0.1 : 1.0;
        }
    }
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pub)->publish(msg);
}

void ROS2Interface::PublishMultibeam2(rclcpp::PublisherBase::SharedPtr pub, Multibeam2* mb) const
{
    uint32_t hRes, vRes;
    mb->getResolution(hRes, vRes);
    glm::vec2 range = mb->getRangeLimits();
    float hFovRad = mb->getHorizontalFOV()/180.f*M_PI;
    float vFovRad = mb->getVerticalFOV()/180.f*M_PI; 
    float hStepAngleRad = hFovRad/(float)(hRes-1);
    //float vStepAngleRad = vFovRad/(float)(vRes-1);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    cloud->header.frame_id = mb->getName();
    cloud->height = cloud->width = 1;

    float* data = (float*)mb->getRangeDataPointer();

    for(uint32_t v=0; v<vRes; ++v)
    {
        uint32_t offset = v*hRes;
        float hAngleRad = -hFovRad/2.f + (0.5f/hRes*hFovRad);
        float vAngleRad = vFovRad/2.f - ((0.5f+v)/vRes*vFovRad);
        
        for(uint32_t h=0; h<hRes; ++h)
        {
            float depth = data[offset + h];
            if(depth > range.x && depth < range.y) // Only publish good points
            { 
                glm::vec3 mbPoint = glm::normalize(glm::vec3(tanf(hAngleRad), tanf(vAngleRad), 1.f)) * depth;
                pcl::PointXYZ pt;
                pt.x = mbPoint.x;
                pt.y = mbPoint.y;
                pt.z = mbPoint.z;
                cloud->push_back(pt);
            }
            hAngleRad += hStepAngleRad;
        }
    }
    
    pcl::PCLPointCloud2 pclMsg;
    pcl::toPCLPointCloud2(*cloud, pclMsg);

    sensor_msgs::msg::PointCloud2 msg;
    msg.header.stamp = nh_->get_clock()->now();
    msg.header.frame_id = mb->getName();
    pcl_conversions::fromPCL(pclMsg, msg);

    try
    {
        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>>(pub)->publish(msg);
    }
    catch (std::runtime_error& e)
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Runtime error whle publishing multibeam data: " << e.what());
    }
}

void ROS2Interface::PublishContact(rclcpp::PublisherBase::SharedPtr pub, Contact* cnt) const
{
    if(cnt->getHistory().size() == 0)
        return;

    ContactPoint cp = cnt->getHistory().back();
    
    //Publish marker message
    visualization_msgs::msg::Marker msg;
    msg.header.frame_id = "world_ned";
    msg.header.stamp = nh_->get_clock()->now();
    msg.ns = cnt->getName();
    msg.id = 0;
    msg.type = visualization_msgs::msg::Marker::ARROW;
    msg.action = visualization_msgs::msg::Marker::ADD;
    msg.pose.position.x = 0.0;
    msg.pose.position.y = 0.0;
    msg.pose.position.z = 0.0;
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = 0.0;
    msg.pose.orientation.w = 1.0;
    msg.points.resize(2);
    msg.points[0].x = cp.locationA.getX();
    msg.points[0].y = cp.locationA.getY();
    msg.points[0].z = cp.locationA.getZ();
    msg.points[1].x = cp.locationA.getX() + cp.normalForceA.getX();
    msg.points[1].y = cp.locationA.getY() + cp.normalForceA.getY();
    msg.points[1].z = cp.locationA.getZ() + cp.normalForceA.getZ();
    msg.color.r = 1.0;
    msg.color.g = 1.0;
    msg.color.b = 0.0;
    msg.color.a = 1.0;
    std::static_pointer_cast<rclcpp::Publisher<visualization_msgs::msg::Marker>>(pub)->publish(msg);
}

void ROS2Interface::PublishUSBL(rclcpp::PublisherBase::SharedPtr pub, rclcpp::PublisherBase::SharedPtr pubInfo, USBL* usbl) const
{
    std::map<uint64_t, BeaconInfo>& beacons = usbl->getBeaconInfo();
    if(beacons.size() == 0)
        return;   

    visualization_msgs::msg::MarkerArray msg;
    visualization_msgs::msg::Marker marker;
    stonefish_ros2::msg::BeaconInfo info;
    
    marker.header.frame_id = usbl->getName();
    marker.header.stamp = nh_->get_clock()->now();
    info.header.frame_id = marker.header.frame_id;
    info.header.stamp = marker.header.stamp;

    marker.ns = usbl->getName();
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = marker.scale.y = marker.scale.z = 0.1;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    for(auto it = beacons.begin(); it!=beacons.end(); ++it)
    {
        marker.id = it->first;
        Vector3 pos = it->second.relPos;
        marker.pose.position.x = pos.getX();
        marker.pose.position.y = pos.getY();
        marker.pose.position.z = pos.getZ();
        msg.markers.push_back(marker);    

        info.beacon_id = it->first;
        info.range = it->second.range;
        info.azimuth = it->second.azimuth;
        info.elevation = it->second.elevation;
        info.relative_position.x = it->second.relPos.getX();
        info.relative_position.y = it->second.relPos.getY();
        info.relative_position.z = it->second.relPos.getZ();
        info.local_orientation.x = it->second.localOri.getX();
        info.local_orientation.y = it->second.localOri.getY();
        info.local_orientation.z = it->second.localOri.getZ();
        info.local_orientation.w = it->second.localOri.getW();
        info.local_depth = it->second.localDepth;
        std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::BeaconInfo>>(pubInfo)->publish(info);
    }
    std::static_pointer_cast<rclcpp::Publisher<visualization_msgs::msg::MarkerArray>>(pub)->publish(msg);
}

void ROS2Interface::PublishTrajectoryState(rclcpp::PublisherBase::SharedPtr pubOdom, rclcpp::PublisherBase::SharedPtr pubIter, AnimatedEntity* anim) const
{
    sf::Trajectory* tr = anim->getTrajectory();
    sf::Transform T = tr->getInterpolatedTransform();
    sf::Vector3 p = T.getOrigin();
    sf::Quaternion q = T.getRotation();
    sf::Vector3 v = tr->getInterpolatedLinearVelocity();
    sf::Vector3 omega = tr->getInterpolatedAngularVelocity();

    //Odometry message
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "world_ned";
    msg.header.stamp = nh_->get_clock()->now();
    msg.child_frame_id = anim->getName();
    msg.pose.pose.position.x = p.x();
    msg.pose.pose.position.y = p.y();
    msg.pose.pose.position.z = p.z();
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();
    msg.twist.twist.linear.x = v.x();
    msg.twist.twist.linear.y = v.y();
    msg.twist.twist.linear.z = v.z();
    msg.twist.twist.angular.x = omega.x();
    msg.twist.twist.angular.y = omega.y();
    msg.twist.twist.angular.z = omega.z();
    std::static_pointer_cast<rclcpp::Publisher<nav_msgs::msg::Odometry>>(pubOdom)->publish(msg);

    //Iteration message
    stonefish_ros2::msg::Int32Stamped msg2;
    msg2.header = msg.header;
    msg2.data = (int32_t)tr->getPlaybackIteration();
    std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::Int32Stamped>>(pubIter)->publish(msg2);
}

void ROS2Interface::PublishEventBasedCamera(rclcpp::PublisherBase::SharedPtr pub, EventBasedCamera* ebc)
{
    //Get access to event texture
    int32_t* data = (int32_t*)ebc->getImageDataPointer();

    //Event array message
    stonefish_ros2::msg::EventArray msg;
    msg.header.frame_id = ebc->getName();
    msg.header.stamp = nh_->get_clock()->now();
    ebc->getResolution(msg.width, msg.height);
    msg.events.resize(ebc->getLastEventCount());
    for(size_t i=0; i<msg.events.size(); ++i)
    {
        //First 4 bytes - pixel coords
        msg.events[i].x = (unsigned int)(data[i*2] >> 16);
        msg.events[i].y = (unsigned int)(data[i*2] & 0xFFFF); 
        //Next 4 bytes - polarity and time
        msg.events[i].ts = msg.header.stamp + rclcpp::Duration(0, abs(data[i*2+1]));
        msg.events[i].polarity = data[i*2+1] > 0;
    }
    std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::EventArray>>(pub)->publish(msg);
}

std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr> ROS2Interface::GenerateCameraMsgPrototypes(Camera* cam, bool depth, const std::string frame_id)
{
    //Image message
    sensor_msgs::msg::Image::SharedPtr img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.frame_id = frame_id != "" ? frame_id : cam->getName();
	cam->getResolution(img->width, img->height);
	img->encoding = depth ? "32FC1" : "rgb8";
	img->is_bigendian = 0;
    img->step = img->width * (depth ? sizeof(float) : 3);
    img->data.resize(img->step * img->height);

	//Camera info message
	sensor_msgs::msg::CameraInfo::SharedPtr info = std::make_shared<sensor_msgs::msg::CameraInfo>();
	info->header.frame_id = cam->getName();
    info->width = img->width;
    info->height = img->height;
    info->binning_x = 0;
    info->binning_y = 0;
    //Distortion
    info->distortion_model = "plumb_bob";
    info->d.resize(5, 0.0);
    //Rectification (for stereo only)
    info->r[0] = 1.0;
    info->r[4] = 1.0;
    info->r[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->getHorizontalFOV()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info->height/(double)info->width * tanhfov2;
    info->k[2] = (double)info->width/2.0; //cx
    info->k[5] = (double)info->height/2.0; //cy
    info->k[0] = info->k[2]/tanhfov2; //fx
    info->k[4] = info->k[5]/tanvfov2; //fy 
    info->k[8] = 1.0;
    //Projection
    info->p[2] = info->k[2]; //cx'
    info->p[6] = info->k[5]; //cy'
    info->p[0] = info->k[0]; //fx';
    info->p[5] = info->k[4]; //fy';
    info->p[3] = 0.0; //Tx - position of second camera from stereo pair
    info->p[7] = 0.0; //Ty;
    info->p[10] = 1.0;
    //ROI
    info->roi.x_offset = 0;
    info->roi.y_offset = 0;
    info->roi.height = info->height;
    info->roi.width = info->width;
    info->roi.do_rectify = false;
	
    return std::make_pair(img, info);
}

std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr> ROS2Interface::GenerateThermalCameraMsgPrototypes(ThermalCamera* cam)
{
    //Image message
    sensor_msgs::msg::Image::SharedPtr img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.frame_id = cam->getName();
    cam->getResolution(img->width, img->height);
    img->encoding = "32FC1";
    img->is_bigendian = 0;
    img->step = img->width * sizeof(float);
    img->data.resize(img->step * img->height);

    //Camera info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    info->header.frame_id = cam->getName();
    info->width = img->width;
    info->height = img->height;
    info->binning_x = 0;
    info->binning_y = 0;
    //Distortion
    info->distortion_model = "plumb_bob";
    info->d.resize(5, 0.0);
    //Rectification (for stereo only)
    info->r[0] = 1.0;
    info->r[4] = 1.0;
    info->r[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->getHorizontalFOV()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info->height/(double)info->width * tanhfov2;
    info->k[2] = (double)info->width/2.0; //cx
    info->k[5] = (double)info->height/2.0; //cy
    info->k[0] = info->k[2]/tanhfov2; //fx
    info->k[4] = info->k[5]/tanvfov2; //fy 
    info->k[8] = 1.0;
    //Projection
    info->p[2] = info->k[2]; //cx'
    info->p[6] = info->k[5]; //cy'
    info->p[0] = info->k[0]; //fx';
    info->p[5] = info->k[4]; //fy';
    info->p[3] = 0.0; //Tx - position of second camera from stereo pair
    info->p[7] = 0.0; //Ty;
    info->p[10] = 1.0;
    //ROI
    info->roi.x_offset = 0;
    info->roi.y_offset = 0;
    info->roi.height = info->height;
    info->roi.width = info->width;
    info->roi.do_rectify = false;

    //Display message
    sensor_msgs::msg::Image::SharedPtr disp = std::make_shared<sensor_msgs::msg::Image>();
    disp->header.frame_id = cam->getName();
    disp->width = img->width;
    disp->height = img->height;
    disp->encoding = "rgb8";
    disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_tuple(img, info, disp);
}

std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr> ROS2Interface::GenerateOpticalFlowCameraMsgPrototypes(OpticalFlowCamera* cam)
{
    //Image message
    sensor_msgs::msg::Image::SharedPtr img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.frame_id = cam->getName();
    cam->getResolution(img->width, img->height);
    img->encoding = "32FC2";
    img->is_bigendian = 0;
    img->step = img->width * 2 * sizeof(float);
    img->data.resize(img->step * img->height);

    //Camera info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    info->header.frame_id = cam->getName();
    info->width = img->width;
    info->height = img->height;
    info->binning_x = 0;
    info->binning_y = 0;
    //Distortion
    info->distortion_model = "plumb_bob";
    info->d.resize(5, 0.0);
    //Rectification (for stereo only)
    info->r[0] = 1.0;
    info->r[4] = 1.0;
    info->r[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->getHorizontalFOV()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info->height/(double)info->width * tanhfov2;
    info->k[2] = (double)info->width/2.0; //cx
    info->k[5] = (double)info->height/2.0; //cy
    info->k[0] = info->k[2]/tanhfov2; //fx
    info->k[4] = info->k[5]/tanvfov2; //fy 
    info->k[8] = 1.0;
    //Projection
    info->p[2] = info->k[2]; //cx'
    info->p[6] = info->k[5]; //cy'
    info->p[0] = info->k[0]; //fx';
    info->p[5] = info->k[4]; //fy';
    info->p[3] = 0.0; //Tx - position of second camera from stereo pair
    info->p[7] = 0.0; //Ty;
    info->p[10] = 1.0;
    //ROI
    info->roi.x_offset = 0;
    info->roi.y_offset = 0;
    info->roi.height = info->height;
    info->roi.width = info->width;
    info->roi.do_rectify = false;

    //Display message
    sensor_msgs::msg::Image::SharedPtr disp = std::make_shared<sensor_msgs::msg::Image>();
    disp->header.frame_id = cam->getName();
    disp->width = img->width;
    disp->height = img->height;
    disp->encoding = "rgb8";
    disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_tuple(img, info, disp);
}

std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr> ROS2Interface::GenerateSegmentationCameraMsgPrototypes(SegmentationCamera* cam)
{
    //Image message
    sensor_msgs::msg::Image::SharedPtr img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.frame_id = cam->getName();
    cam->getResolution(img->width, img->height);
    img->encoding = "16UC1";
    img->is_bigendian = 0;
    img->step = img->width * sizeof(uint16_t);
    img->data.resize(img->step * img->height);

    //Camera info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::make_shared<sensor_msgs::msg::CameraInfo>();
    info->header.frame_id = cam->getName();
    info->width = img->width;
    info->height = img->height;
    info->binning_x = 0;
    info->binning_y = 0;
    //Distortion
    info->distortion_model = "plumb_bob";
    info->d.resize(5, 0.0);
    //Rectification (for stereo only)
    info->r[0] = 1.0;
    info->r[4] = 1.0;
    info->r[8] = 1.0;
    //Intrinsic
    double tanhfov2 = tan(cam->getHorizontalFOV()/180.0*M_PI/2.0);
    double tanvfov2 = (double)info->height/(double)info->width * tanhfov2;
    info->k[2] = (double)info->width/2.0; //cx
    info->k[5] = (double)info->height/2.0; //cy
    info->k[0] = info->k[2]/tanhfov2; //fx
    info->k[4] = info->k[5]/tanvfov2; //fy 
    info->k[8] = 1.0;
    //Projection
    info->p[2] = info->k[2]; //cx'
    info->p[6] = info->k[5]; //cy'
    info->p[0] = info->k[0]; //fx';
    info->p[5] = info->k[4]; //fy';
    info->p[3] = 0.0; //Tx - position of second camera from stereo pair
    info->p[7] = 0.0; //Ty;
    info->p[10] = 1.0;
    //ROI
    info->roi.x_offset = 0;
    info->roi.y_offset = 0;
    info->roi.height = info->height;
    info->roi.width = info->width;
    info->roi.do_rectify = false;

    //Display message
    sensor_msgs::msg::Image::SharedPtr disp = std::make_shared<sensor_msgs::msg::Image>();
    disp->header.frame_id = cam->getName();
    disp->width = img->width;
    disp->height = img->height;
    disp->encoding = "rgb8";
    disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_tuple(img, info, disp);
}

std::pair<std::string, unsigned int> getSonarOutputFormatInfo(SonarOutputFormat fmt)
{
    switch (fmt)
    {
        case SonarOutputFormat::U8:
            return std::make_pair("8UC1", 1);
        case SonarOutputFormat::U16:
            return std::make_pair("16UC1", 2);
        case SonarOutputFormat::F32:
            return std::make_pair("32FC1", 4);
        default:
            throw std::runtime_error("Unsupported sonar output format for ROS message generation");
            break;
    }
}

std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr> ROS2Interface::GenerateFLSMsgPrototypes(FLS* fls)
{
    auto formatInfo = getSonarOutputFormatInfo(fls->getOutputFormat());

    //Image message
    sensor_msgs::msg::Image::SharedPtr img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.frame_id = fls->getName();
    fls->getResolution(img->width, img->height);
    img->encoding = formatInfo.first;
    img->is_bigendian = 0;
    img->step = img->width * formatInfo.second;
    img->data.resize(img->step * img->height);

    //Display message
    sensor_msgs::msg::Image::SharedPtr disp = std::make_shared<sensor_msgs::msg::Image>();
    disp->header.frame_id = fls->getName();
	fls->getDisplayResolution(disp->width, disp->height);
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_pair(img, disp);
}

std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr> ROS2Interface::GenerateSSSMsgPrototypes(SSS* sss)
{
    auto formatInfo = getSonarOutputFormatInfo(sss->getOutputFormat());

    //Image message
    sensor_msgs::msg::Image::SharedPtr img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.frame_id = sss->getName();
    sss->getResolution(img->width, img->height);
    img->encoding = formatInfo.first;
    img->is_bigendian = 0;
    img->step = img->width * formatInfo.second;
    img->data.resize(img->step * img->height);

    //Display message
    sensor_msgs::msg::Image::SharedPtr disp = std::make_shared<sensor_msgs::msg::Image>();
    disp->header.frame_id = sss->getName();
	sss->getDisplayResolution(disp->width, disp->height);
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_pair(img, disp);
}

std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr> ROS2Interface::GenerateMSISMsgPrototypes(MSIS* msis)
{
    auto formatInfo = getSonarOutputFormatInfo(msis->getOutputFormat());

    //Image message
    sensor_msgs::msg::Image::SharedPtr img = std::make_shared<sensor_msgs::msg::Image>();
    img->header.frame_id = msis->getName();
    msis->getResolution(img->width, img->height);
    img->encoding = formatInfo.first;
    img->is_bigendian = 0;
    img->step = img->width * formatInfo.second;
    img->data.resize(img->step * img->height);

    //Display message
    sensor_msgs::msg::Image::SharedPtr disp = std::make_shared<sensor_msgs::msg::Image>();
    disp->header.frame_id = msis->getName();
	msis->getDisplayResolution(disp->width, disp->height);
	disp->encoding = "rgb8";
	disp->is_bigendian = 0;
    disp->step = disp->width * 3;
    disp->data.resize(disp->step * disp->height);
    
    return std::make_pair(img, disp);
}

}