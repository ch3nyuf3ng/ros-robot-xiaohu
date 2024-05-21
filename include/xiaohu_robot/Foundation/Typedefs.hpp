#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_TYPE_HPP
#define XIAOHU_ROBOT_FOUNDATION_TYPE_HPP

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "wpb_home_behaviors/Coord.h"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/GeneralTaskMessage.h"
#include "xiaohu_robot/InspectionTaskMessage.h"
#include "xiaohu_robot/MappingTaskMessage.h"
#include "xiaohu_robot/MedicineDeliveryTaskMessage.h"
#include "xiaohu_robot/NodeControlMessage.h"
#include "xiaohu_robot/StatusAndDescriptionMessage.h"
#include <sound_play/sound_play.h>

namespace xiaohu_robot {
inline namespace Foundation {
inline namespace Typedefs {
using StringMessage = std_msgs::String;
using Float64Message = std_msgs::Float64;
using VelocityCommandMessage = geometry_msgs::Twist;
using CoordinateMessage = geometry_msgs::Pose;
using CoordinateCovarianceStampedMessage = geometry_msgs::PoseWithCovarianceStamped;
using ManipulatorControlMessage = sensor_msgs::JointState;
using ObjectDetectionResultMessasge = wpb_home_behaviors::Coord;

using StringMessagePointer = StringMessage::ConstPtr const&;
using Float64MessagePointer = Float64Message::ConstPtr const&;
using NodeControlMessagePointer = NodeControlMessage::ConstPtr const&;
using ObjectDetectionResultMessasgePointer = ObjectDetectionResultMessasge::ConstPtr const&;
using JoystickMessagePointer = sensor_msgs::Joy::ConstPtr const&;
using CoordinateMessagePointer = CoordinateMessage::ConstPtr const&;
using InspectionTaskMessagePointer = InspectionTaskMessage::ConstPtr const&;
using MappingTaskMessagePointer = MappingTaskMessage::ConstPtr const&;
using MedicineDeliveryTaskMessagePointer = MedicineDeliveryTaskMessage::ConstPtr const&;
using GeneralTaskMessagePointer = GeneralTaskMessage::ConstPtr const&;
using StatusAndDescriptionMessagePointer = StatusAndDescriptionMessage::ConstPtr const&;

using NodeHandle = ros::NodeHandle;
using Publisher = ros::Publisher;
using Subscriber = ros::Subscriber;
using LoopRate = ros::Rate;

using Length = Measurement<UnitLength>;
using Duration = Measurement<UnitDuration>;
using Frequency = Measurement<UnitFrequency>;
using LinearSpeed = Measurement<UnitSpeed>;
using AngularSpeed = Measurement<UnitAngularSpeed>;

using NavigationClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using SoundplayClient = sound_play::SoundClient;
using ServiceClient = ros::ServiceClient;
using Procedure = std_srvs::Empty;

using NavigationGoal = move_base_msgs::MoveBaseGoal;
using GoalState = actionlib::SimpleClientGoalState;
}  // namespace Typedefs
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif
