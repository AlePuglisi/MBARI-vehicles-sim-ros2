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
//  ROS2Interface.h
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 04/10/23.
//  Copyright (c) 2023-2025 Patryk Cieslak. All rights reserved.
//

#ifndef __Stonefish_ROS2Interface__
#define __Stonefish_ROS2Interface__

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "tf2_ros/transform_broadcaster.h"

#include <Stonefish/StonefishCommon.h>

namespace sf
{
    class Accelerometer;
    class Gyroscope;
    class IMU;
    class Pressure;
    class DVL;
    class GPS;
    class INS;
    class Odometry;
    class ForceTorque;
    class RotaryEncoder;
    class Camera;
    class ThermalCamera;
    class OpticalFlowCamera;
    class SegmentationCamera;
    class EventBasedCamera;
    class Multibeam;
    class Profiler;
    class Multibeam2;
    class FLS;
    class SSS;
    class MSIS;
    class Contact;
    class USBL;
    class AnimatedEntity;

    class ROS2Interface
    {
    public:
        ROS2Interface(const std::shared_ptr<rclcpp::Node> nh);
        void PublishTF(std::unique_ptr<tf2_ros::TransformBroadcaster>& br, const sf::Transform& T, const rclcpp::Time& t, const std::string &frame_id, const std::string &child_frame_id) const;
        void PublishAccelerometer(rclcpp::PublisherBase::SharedPtr pub, Accelerometer* acc) const;
        void PublishGyroscope(rclcpp::PublisherBase::SharedPtr pub, Gyroscope* gyro) const;
        void PublishIMU(rclcpp::PublisherBase::SharedPtr pub, IMU* imu) const;
        void PublishPressure(rclcpp::PublisherBase::SharedPtr pub, Pressure* press) const;
        void PublishDVL(rclcpp::PublisherBase::SharedPtr pub, DVL* dvl) const;
        void PublishDVLAltitude(rclcpp::PublisherBase::SharedPtr pub, DVL* dvl) const;
        void PublishGPS(rclcpp::PublisherBase::SharedPtr pub, GPS* gps) const;
        void PublishOdometry(rclcpp::PublisherBase::SharedPtr pub, Odometry* odom) const;
        void PublishINS(rclcpp::PublisherBase::SharedPtr pub, INS* ins) const;
        void PublishINSOdometry(rclcpp::PublisherBase::SharedPtr pub, INS* ins) const;
        void PublishForceTorque(rclcpp::PublisherBase::SharedPtr pub, ForceTorque* ft) const;
        void PublishEncoder(rclcpp::PublisherBase::SharedPtr pub, RotaryEncoder* enc) const;
        void PublishMultibeam(rclcpp::PublisherBase::SharedPtr pub, Multibeam* mb) const;
        void PublishMultibeamPCL(rclcpp::PublisherBase::SharedPtr pub, Multibeam* mb) const;
        void PublishProfiler(rclcpp::PublisherBase::SharedPtr pub, Profiler* prof) const;
        void PublishMultibeam2(rclcpp::PublisherBase::SharedPtr pub, Multibeam2* mb) const;
        void PublishContact(rclcpp::PublisherBase::SharedPtr pub, Contact* cnt) const;
        void PublishUSBL(rclcpp::PublisherBase::SharedPtr pub, rclcpp::PublisherBase::SharedPtr pubInfo, USBL* usbl) const;
        void PublishTrajectoryState(rclcpp::PublisherBase::SharedPtr pubOdom, rclcpp::PublisherBase::SharedPtr pubIter, AnimatedEntity* anim) const;
        void PublishEventBasedCamera(rclcpp::PublisherBase::SharedPtr pub, EventBasedCamera* ebc);

        static std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr> GenerateCameraMsgPrototypes(Camera* cam, bool depth, const std::string frame_id = "");
        static std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr> GenerateThermalCameraMsgPrototypes(ThermalCamera* cam);
        static std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr> GenerateOpticalFlowCameraMsgPrototypes(OpticalFlowCamera* cam);
        static std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr> GenerateSegmentationCameraMsgPrototypes(SegmentationCamera* cam);
        static std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr> GenerateFLSMsgPrototypes(FLS* fls);
        static std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr> GenerateSSSMsgPrototypes(SSS* sss);
        static std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr> GenerateMSISMsgPrototypes(MSIS* msis);

    private:
        std::shared_ptr<rclcpp::Node> nh_;
    };
}
#endif