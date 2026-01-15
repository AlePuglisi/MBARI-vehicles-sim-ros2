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
//  ROS2SimulationManager.cpp
//  stonefish_ros2
//
//  Created by Patryk Cieslak on 02/10/23.
//  Copyright (c) 2023-2025 Patryk Cieslak. All rights reserved.
//

#include "stonefish_ros2/ROS2SimulationManager.h"
#include "stonefish_ros2/ROS2ScenarioParser.h"
#include "stonefish_ros2/ROS2Interface.h"

#include "stonefish_ros2/msg/thruster_state.hpp"
#include "stonefish_ros2/msg/debug_physics.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"

#include <Stonefish/entities/animation/ManualTrajectory.h>
#include <Stonefish/entities/forcefields/Uniform.h>
#include <Stonefish/entities/forcefields/Jet.h>
#include <Stonefish/joints/FixedJoint.h>
#include <Stonefish/sensors/scalar/Pressure.h>
#include <Stonefish/sensors/scalar/DVL.h>
#include <Stonefish/sensors/scalar/Accelerometer.h>
#include <Stonefish/sensors/scalar/Gyroscope.h>
#include <Stonefish/sensors/scalar/IMU.h>
#include <Stonefish/sensors/scalar/GPS.h>
#include <Stonefish/sensors/scalar/ForceTorque.h>
#include <Stonefish/sensors/scalar/RotaryEncoder.h>
#include <Stonefish/sensors/scalar/Odometry.h>
#include <Stonefish/sensors/vision/ColorCamera.h>
#include <Stonefish/sensors/vision/DepthCamera.h>
#include <Stonefish/sensors/vision/ThermalCamera.h>
#include <Stonefish/sensors/vision/OpticalFlowCamera.h>
#include <Stonefish/sensors/vision/SegmentationCamera.h>
#include <Stonefish/sensors/vision/EventBasedCamera.h>
#include <Stonefish/sensors/scalar/Multibeam.h>
#include <Stonefish/sensors/vision/Multibeam2.h>
#include <Stonefish/sensors/vision/FLS.h>
#include <Stonefish/sensors/vision/SSS.h>
#include <Stonefish/sensors/vision/MSIS.h>
#include <Stonefish/sensors/Contact.h>
#include <Stonefish/comms/USBL.h>
#include <Stonefish/comms/OpticalModem.h>
#include <Stonefish/actuators/Push.h>
#include <Stonefish/actuators/SimpleThruster.h>
#include <Stonefish/actuators/Thruster.h>
#include <Stonefish/actuators/Propeller.h>
#include <Stonefish/actuators/Rudder.h>
#include <Stonefish/actuators/SuctionCup.h>
#include <Stonefish/actuators/Motor.h>
#include <Stonefish/actuators/Servo.h>
#include <Stonefish/actuators/VariableBuoyancy.h>
#include <Stonefish/actuators/Light.h>
#include <Stonefish/core/Robot.h>

using namespace std::placeholders;

namespace sf
{

ROS2SimulationManager::ROS2SimulationManager(Scalar stepsPerSecond, std::string scenarioFilePath, const std::shared_ptr<rclcpp::Node>& nh)
	: SimulationManager(stepsPerSecond, Solver::SI, CollisionFilter::EXCLUSIVE), scenarioPath_(scenarioFilePath), nh_(nh)
{
    it_ = std::make_shared<image_transport::ImageTransport>(nh_);
    interface_ = std::make_shared<ROS2Interface>(nh_);
    tf_ = std::make_unique<tf2_ros::TransformBroadcaster>(nh_);
}

ROS2SimulationManager::~ROS2SimulationManager()
{
}

uint64_t ROS2SimulationManager::getSimulationClock() const
{
    rclcpp::Time now = nh_->get_clock()->now(); // Using nh_->get_clock()->now() is WRONG beacause use_sim_time will not work!
    return now.nanoseconds()/1000;
}

void ROS2SimulationManager::SimulationClockSleep(uint64_t us)
{
    rclcpp::Duration duration(0, (uint32_t)us*1000);
    nh_->get_clock()->sleep_for(duration);
}

std::map<std::string, rclcpp::ServiceBase::SharedPtr>& ROS2SimulationManager::getServices()
{
    return srvs_;
}

std::map<std::string, rclcpp::PublisherBase::SharedPtr>& ROS2SimulationManager::getPublishers()
{
    return pubs_;
}
    
std::map<std::string, rclcpp::SubscriptionBase::SharedPtr>& ROS2SimulationManager::getSubscribers()
{
    return subs_;
}

std::map<std::string, image_transport::Publisher>& ROS2SimulationManager::getImagePublishers()
{
    return imgPubs_;
}

std::shared_ptr<image_transport::ImageTransport> ROS2SimulationManager::getImageTransportHandle()
{
    return it_;
}

std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr>>& ROS2SimulationManager::getCameraMsgPrototypes()
{
    return cameraMsgPrototypes_;
}

std::map<std::string, std::tuple<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::CameraInfo::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& ROS2SimulationManager::getDualImageCameraMsgPrototypes()
{
    return dualImageCameraMsgPrototypes_;
}

std::map<std::string, std::pair<sensor_msgs::msg::Image::SharedPtr, sensor_msgs::msg::Image::SharedPtr>>& ROS2SimulationManager::getSonarMsgPrototypes()
{
    return sonarMsgPrototypes_;
}

void ROS2SimulationManager::AddROS2Robot(const std::shared_ptr<ROS2Robot>& robot)
{
    rosRobots_.push_back(robot);
}

bool ROS2SimulationManager::RespawnROS2Robot(const std::string& robotName, const Transform& origin)
{
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->robot_->getName() == robotName)
        {
            rosRobots_[i]->respawnOrigin_ = origin;
            rosRobots_[i]->respawnRequested_ = true;
            return true;
        }
    }
    return false;
}

void ROS2SimulationManager::BuildScenario()
{
    // Run parser
    ROS2ScenarioParser parser(this, nh_);
    bool success = parser.Parse(scenarioPath_);

    // Save log
    std::string logPath = rclcpp::get_logging_directory().string() + "/stonefish_ros2_parser.log";
    bool success2 = parser.SaveLog(logPath);

    if(!success)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Parsing of scenario file '%s' failed!", scenarioPath_.c_str());
        if(success2)
            RCLCPP_ERROR(nh_->get_logger(), "For more information check the parser log file '%s'.", logPath.c_str());
    }

    if(!success2)
        RCLCPP_ERROR(nh_->get_logger(), "Parser log file '%s' could not be saved!", logPath.c_str());

    // Standard services
    srvs_["enable_currents"] = nh_->create_service<std_srvs::srv::Trigger>("enable_currents", std::bind(&ROS2SimulationManager::EnableCurrentsService, this, _1, _2));
    srvs_["disable_currents"] = nh_->create_service<std_srvs::srv::Trigger>("disable_currents", std::bind(&ROS2SimulationManager::DisableCurrentsService, this, _1, _2));
    srvs_["respawn_robot"] = nh_->create_service<stonefish_ros2::srv::Respawn>("respawn_robot", std::bind(&ROS2SimulationManager::RespawnRobotService, this, _1, _2));
}

void ROS2SimulationManager::DestroyScenario()
{
    for(auto it = imgPubs_.begin(); it != imgPubs_.end(); ++it)
        it->second.shutdown();

    pubs_.clear();
    imgPubs_.clear();
    subs_.clear();
    srvs_.clear();
    cameraMsgPrototypes_.clear();
    sonarMsgPrototypes_.clear();
    rosRobots_.clear();

    SimulationManager::DestroyScenario();
}
	
void ROS2SimulationManager::SimulationStepCompleted(Scalar timeStep)
{
    (void)timeStep; // Suppress warning

    ////////////////////////////////////////SENSORS//////////////////////////////////////////////
    unsigned int id = 0;
    Sensor* sensor;
    while((sensor = getSensor(id++)) != nullptr)
    {
        if(!sensor->isNewDataAvailable())
            continue;

        if(sensor->getType() != SensorType::VISION)
        {
            if(pubs_.find(sensor->getName()) == pubs_.end())
                continue;

            switch(((ScalarSensor*)sensor)->getScalarSensorType())
            {
                case ScalarSensorType::ACC:
                    interface_->PublishAccelerometer(pubs_.at(sensor->getName()), (Accelerometer*)sensor);
                    break;

                case ScalarSensorType::GYRO:
                    interface_->PublishGyroscope(pubs_.at(sensor->getName()), (Gyroscope*)sensor);
                    break;

                case ScalarSensorType::IMU:
                    interface_->PublishIMU(pubs_.at(sensor->getName()), (IMU*)sensor);
                    break;

                case ScalarSensorType::ODOM:
                    interface_->PublishOdometry(pubs_.at(sensor->getName()), (Odometry*)sensor);
                    break;

                case ScalarSensorType::DVL:
                {
                    interface_->PublishDVL(pubs_.at(sensor->getName()), (DVL*)sensor);
                    if(pubs_.find(sensor->getName() + "/altitude") != pubs_.end())
                        interface_->PublishDVLAltitude(pubs_.at(sensor->getName() + "/altitude"), (DVL*)sensor);
                }
                    break;

                case ScalarSensorType::INS:
                {
                    interface_->PublishINS(pubs_.at(sensor->getName()), (INS*)sensor);
                    if(pubs_.find(sensor->getName() + "/odometry") != pubs_.end())
                        interface_->PublishINSOdometry(pubs_.at(sensor->getName() + "/odometry"), (INS*)sensor);
                }
                    break;

                case ScalarSensorType::GPS:
                    interface_->PublishGPS(pubs_.at(sensor->getName()), (GPS*)sensor);
                    break;

                case ScalarSensorType::PRESSURE:
                    interface_->PublishPressure(pubs_.at(sensor->getName()), (Pressure*)sensor);
                    break;

                case ScalarSensorType::FT:
                    interface_->PublishForceTorque(pubs_.at(sensor->getName()), (ForceTorque*)sensor);
                    break;

                case ScalarSensorType::ENCODER:
                    interface_->PublishEncoder(pubs_.at(sensor->getName()), (RotaryEncoder*)sensor);
                    break;

                case ScalarSensorType::MULTIBEAM:
                {
                    interface_->PublishMultibeam(pubs_.at(sensor->getName()), (Multibeam*)sensor);
                    if(pubs_.find(sensor->getName() + "/pcl") != pubs_.end())
                        interface_->PublishMultibeamPCL(pubs_.at(sensor->getName() + "/pcl"), (Multibeam*)sensor);
                }
                    break;

                case ScalarSensorType::PROFILER:
                    interface_->PublishProfiler(pubs_.at(sensor->getName()), (Profiler*)sensor);
                    break;

                default:
                    break;
            }
        }

        sensor->MarkDataOld();
    }

    ///////////////////////////////////////COMMS///////////////////////////////////////////////////
    id = 0;
    Comm* comm;
    while((comm = getComm(id++)) != nullptr)
    {
        if(pubs_.find(comm->getName()) == pubs_.end())
            continue;

        switch(comm->getType())
        {
            case CommType::ACOUSTIC:
            {
                std_msgs::msg::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->getName())
                    )->publish(msg);
                }
            }
                break;

            case CommType::USBL:
            {
                if(comm->isNewDataAvailable())
                {
                    interface_->PublishUSBL(pubs_.at(comm->getName()), pubs_.at(comm->getName() + "/beacon_info"), (USBL*)comm);
                    comm->MarkDataOld();
                }

                std_msgs::msg::String msg;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->getName() + "/received_data")
                    )->publish(msg);
                }
            }
                break;

            case CommType::OPTICAL:
            {
                std_msgs::msg::Float64 msg;
                msg.data = ((OpticalModem*)comm)->getReceptionQuality();
                std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(
                    pubs_.at(comm->getName())
                )->publish(msg);

                std_msgs::msg::String msg2;
                std::shared_ptr<CommDataFrame> message;
                while ((message = comm->ReadMessage()) != nullptr)
                {
                    msg2.data = std::string(message->data.begin(), message->data.end());
                    std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::String>>(
                        pubs_.at(comm->getName() + "/received_data")
                    )->publish(msg2);
                }
            }
                break;

            default:
                break;
        }
    }

    //////////////////////////////////////TRAJECTORIES/////////////////////////////////////////////
    id = 0;
    Entity* ent;
    while((ent = getEntity(id++)) != nullptr)
    {
        if(ent->getType() == EntityType::ANIMATED)
        {
            if(pubs_.find(ent->getName() + "/odometry") == pubs_.end())
                continue;

            interface_->PublishTrajectoryState(pubs_.at(ent->getName() + "/odometry"), pubs_.at(ent->getName() + "/iteration"), (AnimatedEntity*)ent);
        }
    }

    //////////////////////////////////////CONTACTS/////////////////////////////////////////////////
    id = 0;
    Contact* cnt;
    while((cnt = getContact(id++)) != nullptr)
    {
        if(!cnt->isNewDataAvailable())
            continue;

        if(pubs_.find(cnt->getName()) != pubs_.end())
        {
            interface_->PublishContact(pubs_[cnt->getName()], cnt);
            cnt->MarkDataOld();
        }
    }

    //////////////////////////////////////WORLD TRANSFORMS/////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->publishBaseLinkTransform_)
            interface_->PublishTF(tf_, rosRobots_[i]->robot_->getTransform(), nh_->get_clock()->now(), "world_ned", rosRobots_[i]->robot_->getName() + "/base_link");
    }
    
    //////////////////////////////////////SERVOS(JOINTS)/////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/motors") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = nh_->get_clock()->now();
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            
            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::MOTOR)
                {
                    Motor* mtr = (Motor*)actuator;
                    msg.name.push_back(mtr->getJointName());
                    msg.position.push_back(mtr->getAngle());
                    msg.velocity.push_back(mtr->getAngularVelocity());
                    msg.effort.push_back(mtr->getTorque());
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/motors")
                )->publish(msg);
        }

        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/servos") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = nh_->get_clock()->now();
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            
            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::SERVO)
                {
                    Servo* srv = (Servo*)actuator;
                    msg.name.push_back(srv->getJointName());
                    msg.position.push_back(srv->getPosition());
                    msg.velocity.push_back(srv->getVelocity());
                    msg.effort.push_back(srv->getEffort());
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/servos")
                )->publish(msg);
        }

        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/rudders") != pubs_.end())
        {
            unsigned int aID = 0;
            Actuator* actuator;
            sensor_msgs::msg::JointState msg;
            msg.header.stamp = nh_->get_clock()->now();
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            
            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::RUDDER)
                {
                    Rudder* rudder = (Rudder*)actuator;
                    msg.name.push_back(rudder->getName());
                    msg.position.push_back(rudder->getSetpoint());
                    msg.velocity.push_back(0.0);
                    msg.effort.push_back(0.0);
                }
            }
            if(msg.name.size() > 0)
                std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/rudders")
                )->publish(msg);
        }

        if(rosRobots_[i]->thrusterSetpoints_.size() != 0
           && pubs_.find(rosRobots_[i]->robot_->getName() + "/thrusters") != pubs_.end())
        {
            unsigned int aID = 0;
            unsigned int thID = 0;
            Actuator* actuator;
            stonefish_ros2::msg::ThrusterState msg;
            msg.header.stamp = nh_->get_clock()->now();
            msg.header.frame_id = rosRobots_[i]->robot_->getName();
            msg.setpoint.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.rpm.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.thrust.resize(rosRobots_[i]->thrusterSetpoints_.size());
            msg.torque.resize(rosRobots_[i]->thrusterSetpoints_.size());

            while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
            {
                if(actuator->getType() == ActuatorType::THRUSTER)
                {
                    Thruster* th = (Thruster*)actuator;
                    msg.setpoint[thID] = th->getSetpoint();
                    msg.rpm[thID] = th->getOmega()/(Scalar(2)*M_PI)*Scalar(60);
                    msg.thrust[thID] = th->getThrust();
                    msg.torque[thID] = th->getTorque();
                    ++thID;

                    if(thID == rosRobots_[i]->thrusterSetpoints_.size())
                        break;
                }
                else if(actuator->getType() == ActuatorType::SIMPLE_THRUSTER)
                {
                    SimpleThruster* th = (SimpleThruster*)actuator;
                    msg.setpoint[thID] = th->getThrustSetpoint();
                    msg.rpm[thID] = 0.0;
                    msg.thrust[thID] = th->getThrust();
                    msg.torque[thID] = th->getTorque();
                    ++thID;

                    if(thID == rosRobots_[i]->thrusterSetpoints_.size())
                        break;
                }
            }
            std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::ThrusterState>>(
                    pubs_.at(rosRobots_[i]->robot_->getName() + "/thrusters")
                )->publish(msg);
        }
    }

    //////////////////////////////////////////////ACTUATORS//////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        unsigned int aID = 0;
        Actuator* actuator;
        unsigned int thID = 0;
        unsigned int propID = 0;
        unsigned int rudderID = 0;

        while((actuator = rosRobots_[i]->robot_->getActuator(aID++)) != nullptr)
        {
            switch(actuator->getType())
            {
                case ActuatorType::PUSH:
                {
                    Push* push = (Push*)actuator;    
                    if(rosRobots_[i]->thrusterSetpointsChanged_)
                    {
                        push->setForce(rosRobots_[i]->thrusterSetpoints_[thID++]);
                    }

                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = nh_->get_clock()->now();
                        msg.header.frame_id = push->getName();
                        msg.wrench.force.x = push->getForce();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::THRUSTER:
                {
                    Thruster* th = ((Thruster*)actuator);
                    if(rosRobots_[i]->thrusterSetpointsChanged_)
                    {
                        th->setSetpoint(rosRobots_[i]->thrusterSetpoints_[thID++]);
                    }
                        
                    auto it = pubs_.find(actuator->getName()+"/wrench");
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = nh_->get_clock()->now();
                        msg.header.frame_id = th->getName();
                        msg.wrench.force.x = th->getThrust();
                        msg.wrench.torque.x = th->getTorque();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }            

                    it = pubs_.find(actuator->getName()+"/joint_state");
                    if(it != pubs_.end())
                    {
                        //Publish propeller rotation for visualization
                        sensor_msgs::msg::JointState msg;
                        msg.header.stamp = nh_->get_clock()->now();
                        msg.header.frame_id = th->getName();
                        msg.name.push_back(th->getName()+"/propeller");
                        msg.position.push_back(th->getAngle()/Scalar(100)); // Scaled for visualisation 
                        msg.velocity.push_back(th->getOmega());
                        msg.effort.push_back(th->getThrust());
                        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::SIMPLE_THRUSTER:
                {
                    SimpleThruster* th = ((SimpleThruster*)actuator);
                    if(rosRobots_[i]->thrusterSetpointsChanged_)
                    {
                        th->setSetpoint(rosRobots_[i]->thrusterSetpoints_[thID++], Scalar(0));
                    }
                        
                    auto it = pubs_.find(actuator->getName()+"/wrench");
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = nh_->get_clock()->now();
                        msg.header.frame_id = th->getName();
                        msg.wrench.force.x = th->getThrust();
                        msg.wrench.torque.x = th->getTorque();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }            

                    it = pubs_.find(actuator->getName()+"/joint_state");
                    if(it != pubs_.end())
                    {
                        //Publish propeller rotation for visualization
                        sensor_msgs::msg::JointState msg;
                        msg.header.stamp = nh_->get_clock()->now();
                        msg.header.frame_id = th->getName();
                        msg.name.push_back(th->getName()+"/propeller");
                        msg.position.push_back(th->getAngle());  
                        std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::JointState>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::PROPELLER:
                {
                    Propeller* prop = (Propeller*)actuator;
                    if(rosRobots_[i]->propellerSetpointsChanged_)
                    {
                        prop->setSetpoint(rosRobots_[i]->propellerSetpoints_[propID++]);
                    }

                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        geometry_msgs::msg::WrenchStamped msg;
                        msg.header.stamp = nh_->get_clock()->now();
                        msg.header.frame_id = prop->getName();
                        msg.wrench.force.x = prop->getThrust();
                        msg.wrench.torque.x = prop->getTorque();
                        std::static_pointer_cast<rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::RUDDER:
                {
                    if(rosRobots_[i]->rudderSetpointsChanged_)
                    {
                        ((Rudder*)actuator)->setSetpoint(rosRobots_[i]->rudderSetpoints_[rudderID++]);
                    }
                }
                    break;

                case ActuatorType::MOTOR:
                {
                    if(rosRobots_[i]->servoSetpoints_.size() == 0)
                        continue;

                    auto it = rosRobots_[i]->servoSetpoints_.find(((Motor*)actuator)->getJointName());
                    if(it != rosRobots_[i]->servoSetpoints_.end())
                    {
                        if(it->second.first == ServoControlMode::TORQUE)
                        {
                            ((Motor*)actuator)->setCommand(it->second.second);
                        }
                    }
                }
                    break;

                case ActuatorType::SERVO:
                {
                    if(rosRobots_[i]->servoSetpoints_.size() == 0)
                        continue;

                    auto it = rosRobots_[i]->servoSetpoints_.find(((Servo*)actuator)->getJointName());
                    if(it != rosRobots_[i]->servoSetpoints_.end())
                    {
                        if(it->second.first == ServoControlMode::VELOCITY)
                        {
                            ((Servo*)actuator)->setControlMode(ServoControlMode::VELOCITY);
                            ((Servo*)actuator)->setDesiredVelocity(it->second.second);
                        }
                        else if(it->second.first == ServoControlMode::POSITION)
                        {
                            ((Servo*)actuator)->setControlMode(ServoControlMode::POSITION);
                            ((Servo*)actuator)->setDesiredPosition(it->second.second);
                        }
                    }
                }
                    break;

                case ActuatorType::VBS:
                {
                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        std_msgs::msg::Float64 msg;
                        msg.data = ((VariableBuoyancy*)actuator)->getLiquidVolume();
                        std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Float64>>(it->second)->publish(msg);
                    }
                }
                    break;

                case ActuatorType::SUCTION_CUP:
                {
                    auto it = pubs_.find(actuator->getName());
                    if(it != pubs_.end())
                    {
                        std_msgs::msg::Bool msg;
                        msg.data = ((SuctionCup*)actuator)->getPump();
                        std::static_pointer_cast<rclcpp::Publisher<std_msgs::msg::Bool>>(it->second)->publish(msg);
                    }
                }
                    break;

                default:
                    break;
            }
        }
        //Reset change flags
        rosRobots_[i]->thrusterSetpointsChanged_ = false;
        rosRobots_[i]->propellerSetpointsChanged_ = false;
        rosRobots_[i]->rudderSetpointsChanged_ = false;
    }

    /////////////////////////////////// RESPAWN REQUESTS ///////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(rosRobots_[i]->respawnRequested_)
        {
            rosRobots_[i]->robot_->Respawn(this, rosRobots_[i]->respawnOrigin_);
            rosRobots_[i]->respawnRequested_ = false;
        }
    }    

    /////////////////////////////////// DEBUG //////////////////////////////////////////
    for(size_t i=0; i<rosRobots_.size(); ++i)
    {
        if(pubs_.find(rosRobots_[i]->robot_->getName() + "/debug/physics") != pubs_.end())
        {
            Robot* r = rosRobots_[i]->robot_;
            size_t lID = 0;
            SolidEntity* link;
            stonefish_ros2::msg::DebugPhysics msg;
            msg.header.stamp = nh_->get_clock()->now();
            
            auto debugPub = std::static_pointer_cast<rclcpp::Publisher<stonefish_ros2::msg::DebugPhysics>>(
                pubs_.at(r->getName() + "/debug/physics")
            );

            while((link = r->getLink(lID++)) != nullptr)
            {
                msg.header.frame_id = link->getName();
                
                msg.mass = link->getMass();
                msg.volume = link->getVolume();
                msg.surface = link->getSurface();
                Vector3 inertia = link->getInertia();
                msg.inertia.x = inertia.getX();
                msg.inertia.y = inertia.getY();
                msg.inertia.z = inertia.getZ();

                Vector3 cog = -link->getCG2OTransform().getOrigin();
                msg.cog.x = cog.getX();
                msg.cog.y = cog.getY();
                msg.cog.z = cog.getZ();
                
                Vector3 cob = link->getCG2OTransform() * link->getCB();
                msg.cob.x = cob.getX();
                msg.cob.y = cob.getY();
                msg.cob.z = cob.getZ();
                
                Matrix3 toOrigin = link->getOTransform().getBasis().inverse();

                Vector3 vel = toOrigin * link->getLinearVelocity();
                Vector3 avel = toOrigin * link->getAngularVelocity();
                msg.velocity.linear.x = vel.getX();
                msg.velocity.linear.y = vel.getY();
                msg.velocity.linear.z = vel.getZ();
                msg.velocity.angular.x = avel.getX();
                msg.velocity.angular.y = avel.getY();
                msg.velocity.angular.z = avel.getZ();

                Vector3 Fb, Tb, Fd, Td, Ff, Tf;
                link->getHydrodynamicForces(Fb, Tb, Fd, Td, Ff, Tf);            
                Vector3 Cd, Cf;
                link->getHydrodynamicCoefficients(Cd, Cf);
                msg.damping_coeff.x = Cd.getX();
                msg.damping_coeff.y = Cd.getY();
                msg.damping_coeff.z = Cd.getZ();
                msg.skin_friction_coeff.x = Cf.getX();
                msg.skin_friction_coeff.y = Cf.getY();
                msg.skin_friction_coeff.z = Cf.getZ();

                // Fb = toOrigin * Fb;
                // Tb = toOrigin * Tb;
                msg.buoyancy.force.x = Fb.getX();
                msg.buoyancy.force.y = Fb.getY();
                msg.buoyancy.force.z = Fb.getZ();
                msg.buoyancy.torque.x = Tb.getX();
                msg.buoyancy.torque.y = Tb.getY();
                msg.buoyancy.torque.z = Tb.getZ();

                Fd = toOrigin * Fd;
                Td = toOrigin * Td;
                msg.damping.force.x = Fd.getX();
                msg.damping.force.y = Fd.getY();
                msg.damping.force.z = Fd.getZ();
                msg.damping.torque.x = Td.getX();
                msg.damping.torque.y = Td.getY();
                msg.damping.torque.z = Td.getZ();

                Ff = toOrigin * Ff;
                Tf = toOrigin * Tf;
                msg.skin_friction.force.x = Ff.getX();
                msg.skin_friction.force.y = Ff.getY();
                msg.skin_friction.force.z = Ff.getZ();
                msg.skin_friction.torque.x = Tf.getX();
                msg.skin_friction.torque.y = Tf.getY();
                msg.skin_friction.torque.z = Tf.getZ();

                msg.wetted_surface = link->getWettedSurface();
                msg.submerged_volume = link->getSubmergedVolume();
                
                debugPub->publish(msg);
            }
        }
    }
}

void ROS2SimulationManager::ColorCameraImageReady(ColorCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = cameraMsgPrototypes_[cam->getName()].first;
    img->header.stamp = nh_->get_clock()->now();
    memcpy(img->data.data(), (uint8_t*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = cameraMsgPrototypes_[cam->getName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
}

void ROS2SimulationManager::DepthCameraImageReady(DepthCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = cameraMsgPrototypes_[cam->getName()].first;
    img->header.stamp = nh_->get_clock()->now();
    memcpy(img->data.data(), (float*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = cameraMsgPrototypes_[cam->getName()].second;
    info->header.stamp = img->header.stamp;

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
}

void ROS2SimulationManager::ThermalCameraImageReady(ThermalCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img->header.stamp = nh_->get_clock()->now();
    memcpy(img->data.data(), (float*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->getName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->getDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
    imgPubs_.at(cam->getName()+"/display").publish(img2);
}

void ROS2SimulationManager::OpticalFlowCameraImageReady(OpticalFlowCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img->header.stamp = nh_->get_clock()->now();
    memcpy(img->data.data(), (float*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->getName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->getDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
    imgPubs_.at(cam->getName()+"/display").publish(img2);
}

void ROS2SimulationManager::SegmentationCameraImageReady(SegmentationCamera* cam)
{
    //Fill in the image message
    sensor_msgs::msg::Image::SharedPtr img = std::get<0>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img->header.stamp = nh_->get_clock()->now();
    memcpy(img->data.data(), (uint16_t*)cam->getImageDataPointer(), img->step * img->height);

    //Fill in the info message
    sensor_msgs::msg::CameraInfo::SharedPtr info = std::get<1>(dualImageCameraMsgPrototypes_[cam->getName()]);
    info->header.stamp = img->header.stamp;

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr img2 = std::get<2>(dualImageCameraMsgPrototypes_[cam->getName()]);
    img2->header.stamp = img->header.stamp;
    memcpy(img2->data.data(), (uint8_t*)cam->getDisplayDataPointer(), img2->step * img2->height);

    //Publish messages
    imgPubs_.at(cam->getName()).publish(img);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::CameraInfo>>(pubs_.at(cam->getName() + "/info"))->publish(*info);
    imgPubs_.at(cam->getName()+"/display").publish(img2);
}

void ROS2SimulationManager::EventBasedCameraOutputReady(EventBasedCamera* cam)
{
    interface_->PublishEventBasedCamera(pubs_.at(cam->getName()), cam);
}

void ROS2SimulationManager::Multibeam2ScanReady(Multibeam2* mb)
{
    interface_->PublishMultibeam2(pubs_.at(mb->getName()), mb);
}

void ROS2SimulationManager::FLSScanReady(FLS* fls)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[fls->getName()].first;
    img->header.stamp = nh_->get_clock()->now();
    memcpy(img->data.data(), fls->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[fls->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), fls->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(fls->getName()).publish(img);
    imgPubs_.at(fls->getName() + "/display").publish(disp);

}

void ROS2SimulationManager::SSSScanReady(SSS* sss)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[sss->getName()].first;
    img->header.stamp = nh_->get_clock()->now();
    memcpy(img->data.data(), sss->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[sss->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), sss->getDisplayDataPointer(), disp->step * disp->height);

    //Publish messages
    imgPubs_.at(sss->getName()).publish(img);
    imgPubs_.at(sss->getName() + "/display").publish(disp);
}

void ROS2SimulationManager::MSISScanReady(MSIS* msis)
{
    //Fill in the data message
    sensor_msgs::msg::Image::SharedPtr img = sonarMsgPrototypes_[msis->getName()].first;
    img->header.stamp = nh_->get_clock()->now();
    memcpy(img->data.data(), msis->getImageDataPointer(), img->step * img->height);

    //Fill in the display message
    sensor_msgs::msg::Image::SharedPtr disp = sonarMsgPrototypes_[msis->getName()].second;
    disp->header.stamp = img->header.stamp;
    memcpy(disp->data.data(), msis->getDisplayDataPointer(), disp->step * disp->height);

    //Fill in the laser scan message
    Scalar currentAngle = (msis->getCurrentRotationStep() * msis->getRotationStepAngle()) * M_PI / 180.0;
    unsigned int currentBeamIndex = msis->getCurrentBeamIndex();

    sensor_msgs::msg::LaserScan laserscan;
    laserscan.header.stamp = img->header.stamp;
    laserscan.header.frame_id = msis->getName();
    laserscan.angle_min = currentAngle;
    laserscan.angle_max = currentAngle;
    laserscan.angle_increment = 0.0;
    laserscan.time_increment = 0.0;
    laserscan.range_min = msis->getRangeMin();
    laserscan.range_max = msis->getRangeMax();
    laserscan.ranges.resize(img->height);
    laserscan.intensities.resize(img->height);

    for(unsigned int i=0; i<img->height; ++i)
    {
        laserscan.ranges[i] = (laserscan.range_max - laserscan.range_min) * (img->height-1-i)/Scalar(img->height-1) + laserscan.range_min;
        laserscan.intensities[i] = img->data[img->step * i + currentBeamIndex];
    }

    //Publish messages
    imgPubs_.at(msis->getName()).publish(img);
    imgPubs_.at(msis->getName() + "/display").publish(disp);
    std::static_pointer_cast<rclcpp::Publisher<sensor_msgs::msg::LaserScan>>(pubs_.at(msis->getName() + "/beam"))->publish(laserscan);
}

void ROS2SimulationManager::EnableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                           std_srvs::srv::Trigger::Response::SharedPtr res)
{
    (void)req;
    getOcean()->EnableCurrents();
    res->message = "Ocean current simulation enabled.";
    res->success = true;
}

void ROS2SimulationManager::DisableCurrentsService(const std_srvs::srv::Trigger::Request::SharedPtr req, 
                                            std_srvs::srv::Trigger::Response::SharedPtr res)
{   
    (void)req;
    getOcean()->DisableCurrents();
    res->message = "Ocean current simulation disabled.";
    res->success = true;
}

void ROS2SimulationManager::RespawnRobotService(const stonefish_ros2::srv::Respawn::Request::SharedPtr req, 
                             stonefish_ros2::srv::Respawn::Response::SharedPtr res)
{
    Vector3 p(req->origin.position.x, req->origin.position.y, req->origin.position.z);
    Quaternion q(req->origin.orientation.x, req->origin.orientation.y, req->origin.orientation.z, req->origin.orientation.w);
    Transform origin(q, p);
    
    if(RespawnROS2Robot(req->name, origin))
    {
        res->message = "Robot respawned.";
        res->success = true;
    }
    else    
    {
        res->message = "Robot not found.";
        res->success= false;
    }
}

void ROS2SimulationManager::UniformVFCallback(const geometry_msgs::msg::Vector3::SharedPtr msg, Uniform* vf)
{
    vf->setVelocity(Vector3(msg->x, msg->y, msg->z));   
}

void ROS2SimulationManager::JetVFCallback(const std_msgs::msg::Float64::SharedPtr msg, Jet* vf)
{
    vf->setOutletVelocity(msg->data);
}

void ROS2SimulationManager::ActuatorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Actuator* act)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(act->getType())
    {
        case ActuatorType::PUSH:
        case ActuatorType::SIMPLE_THRUSTER:
        case ActuatorType::THRUSTER:
        case ActuatorType::PROPELLER:
        case ActuatorType::VBS:
        case ActuatorType::LIGHT:
            ((LinkActuator*)act)->setRelativeActuatorFrame(T);
            break;

        default:
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Live update of origin frame of actuator '" << act->getName() << "' not supported!");
            break;
    }
}

void ROS2SimulationManager::TrajectoryCallback(const nav_msgs::msg::Odometry::SharedPtr msg, ManualTrajectory* tr)
{
    Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    Vector3 p(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Vector3 v(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    Vector3 omega(msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    tr->setTransform(Transform(q, p));
    tr->setLinearVelocity(v);
    tr->setAngularVelocity(omega);
}

void ROS2SimulationManager::SimpleThrusterCallback(const std_msgs::msg::Float64::SharedPtr msg, SimpleThruster* th)
{
    th->setSetpoint(msg->data, Scalar(0));
}

void ROS2SimulationManager::ThrusterCallback(const std_msgs::msg::Float64::SharedPtr msg, Thruster* th)
{
    th->setSetpoint(msg->data);
}

void ROS2SimulationManager::PropellerCallback(const std_msgs::msg::Float64::SharedPtr msg, Propeller* prop)
{
    prop->setSetpoint(msg->data);
}

void ROS2SimulationManager::PushCallback(const std_msgs::msg::Float64::SharedPtr msg, Push* push)
{
    push->setForce(msg->data);
}

void ROS2SimulationManager::VBSCallback(const std_msgs::msg::Float64::SharedPtr msg, VariableBuoyancy* act)
{
    act->setFlowRate(msg->data);
}

void ROS2SimulationManager::CommCallback(const std_msgs::msg::String::SharedPtr msg, Comm* comm)
{
    comm->SendMessage(msg->data);
}

void ROS2SimulationManager::SuctionCupService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                            std_srvs::srv::SetBool::Response::SharedPtr res, SuctionCup* suction)
{
    suction->setPump(req->data);
    if(req->data)
        res->message = "Pump turned on.";
    else 
        res->message = "Pump turned off.";
    res->success = true;
}

void ROS2SimulationManager::SensorService(const std_srvs::srv::SetBool::Request::SharedPtr req,
                                        std_srvs::srv::SetBool::Response::SharedPtr res, Sensor* sens)
{
    sens->setEnabled(req->data);
    if(req->data)
        res->message = "Sensor turned on.";
    else
        res->message = "Sensor turned off.";
    res->success = true;    
}

void ROS2SimulationManager::SensorOriginCallback(const geometry_msgs::msg::Transform::SharedPtr msg, Sensor* sens)
{
    Transform T;
    T.setOrigin(Vector3(msg->translation.x, msg->translation.y, msg->translation.z));
    T.setRotation(Quaternion(msg->rotation.x, msg->rotation.y, msg->rotation.z, msg->rotation.w));

    switch(sens->getType())
    {
        case SensorType::LINK:
            ((LinkSensor*)sens)->setRelativeSensorFrame(T);
            break;

        case SensorType::VISION:
            ((VisionSensor*)sens)->setRelativeSensorFrame(T);
            break;

        default:
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Live update of origin frame of sensor '" << sens->getName() << "' not supported!");
            break;
    }
}

void ROS2SimulationManager::FLSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, FLS* fls)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0 || req->range_min >= req->range_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        fls->setRangeMax(req->range_max);
        fls->setRangeMin(req->range_min);
        fls->setGain(req->gain);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::SSSService(const stonefish_ros2::srv::SonarSettings::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings::Response::SharedPtr res, SSS* sss)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0 || req->range_min >= req->range_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        sss->setRangeMax(req->range_max);
        sss->setRangeMin(req->range_min);
        sss->setGain(req->gain);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::MSISService(const stonefish_ros2::srv::SonarSettings2::Request::SharedPtr req,
                                    stonefish_ros2::srv::SonarSettings2::Response::SharedPtr res, MSIS* msis)
{
    if(req->range_min <= 0 || req->range_max <= 0 || req->gain <= 0
       || req->range_min >= req->range_max
       || req->rotation_min < -180.0
       || req->rotation_max > 180.0
       || req->rotation_min >= req->rotation_max)
    {
        res->success = false;
        res->message = "Wrong sonar settings!";
    }
    else
    {
        msis->setRangeMax(req->range_max);
        msis->setRangeMin(req->range_min);
        msis->setGain(req->gain);
        msis->setRotationLimits(req->rotation_min, req->rotation_max);
        res->success = true;
        res->message = "New sonar settings applied.";
    }
}

void ROS2SimulationManager::ThrustersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->thrusterSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of thruster setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->thrusterSetpoints_.size(); ++i)
        robot->thrusterSetpoints_[i] = msg->data[i];
    robot->thrusterSetpointsChanged_ = true;
}

void ROS2SimulationManager::PropellersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->propellerSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of propeller setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->propellerSetpoints_.size(); ++i)
        robot->propellerSetpoints_[i] = msg->data[i];
    robot->propellerSetpointsChanged_ = true;
}

void ROS2SimulationManager::RuddersCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->data.size() != robot->rudderSetpoints_.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong number of rudder setpoints for robot: " << robot->robot_->getName());
        return;
    }
    for(size_t i=0; i<robot->rudderSetpoints_.size(); ++i)
        robot->rudderSetpoints_[i] = msg->data[i];
    robot->rudderSetpointsChanged_ = true;
}

void ROS2SimulationManager::ServosCallback(const sensor_msgs::msg::JointState::SharedPtr msg, std::shared_ptr<ROS2Robot> robot)
{
    if(msg->name.size() == 0)
    {
        RCLCPP_ERROR(nh_->get_logger(), "Desired joint state message is missing joint names!");
        return;
    }

    if(msg->position.size() > 0)
    {
        for(size_t i=0; i<msg->position.size(); ++i)
        {
            try
            {
                robot->servoSetpoints_.at(msg->name[i]) = std::make_pair(ServoControlMode::POSITION, (Scalar)msg->position[i]);
            }
            catch(const std::out_of_range& e)
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->velocity.size() > 0)
    {
        for(size_t i=0; i<msg->velocity.size(); ++i)
        {
            try
            {
                robot->servoSetpoints_.at(msg->name[i]) = std::make_pair(ServoControlMode::VELOCITY, (Scalar)msg->velocity[i]);
            }
            catch(const std::out_of_range& e)
            {
                RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name in desired joint state message: " << msg->name[i]);
            }
        }
    }
    else if(msg->effort.size() > 0)
    {
        RCLCPP_ERROR(nh_->get_logger(), "No effort control mode implemented in simulation!");
    }
}

void ROS2SimulationManager::JointCallback(const std_msgs::msg::Float64::SharedPtr msg,  std::shared_ptr<ROS2Robot> robot, 
                                                            ServoControlMode mode, const std::string& jointName)
{
    try
    {
        robot->servoSetpoints_.at(jointName) = std::make_pair(mode, (Scalar)msg->data);
    }
    catch(const std::out_of_range& e)
    {
        RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name: " << jointName);
    }
}

void ROS2SimulationManager::JointGroupCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg,  std::shared_ptr<ROS2Robot> robot, 
                                                            ServoControlMode mode, const std::vector<std::string>& jointNames)
{
    if(msg->data.size() != jointNames.size())
    {
        RCLCPP_ERROR_STREAM(nh_->get_logger(), "Wrong size of joint group message! Required: " << jointNames.size() << " Received: " << msg->data.size());
        return;
    }
    for(size_t i=0; i<jointNames.size(); ++i)
    {
        try
        {
            robot->servoSetpoints_.at(jointNames[i]) = std::make_pair(mode, (Scalar)msg->data[i]);
        }
        catch(const std::out_of_range& e)
        {
            RCLCPP_WARN_STREAM(nh_->get_logger(), "Invalid joint name: " << jointNames[i]);
        }
    }
}

void ROS2SimulationManager::GlueService(const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res, FixedJoint* fix)
{
    if(req->data)
    {
        fix->RemoveFromSimulation(this);
        fix->UpdateDefinition();
        fix->AddToSimulation(this);
        res->message = "Glue activated.";
    }
    else
    {
        fix->RemoveFromSimulation(this);  
        res->message = "Glue deactivated.";
    }
    res->success = true;
}

void ROS2SimulationManager::LightService(const std_srvs::srv::SetBool::Request::SharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res, Light* light)
{
    light->Switch(req->data);
    if(req->data)
        res->message = "Light turned on.";
    else
        res->message = "Light turned off.";
    res->success = true;
}

}