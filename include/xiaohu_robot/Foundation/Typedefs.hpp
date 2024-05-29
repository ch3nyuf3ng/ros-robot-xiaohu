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
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "wpb_home_behaviors/Coord.h"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/InspectionTaskRequest.h"
#include "xiaohu_robot/InspectionTaskResult.h"
#include "xiaohu_robot/LegacyGeneralTaskRequest.h"
#include "xiaohu_robot/LegacyGeneralTaskResult.h"
#include "xiaohu_robot/MappingTaskRequest.h"
#include "xiaohu_robot/MappingTaskResult.h"
#include "xiaohu_robot/MedicineDeliveryTaskRequest.h"
#include "xiaohu_robot/MedicineDeliveryTaskResult.h"
#include "xiaohu_robot/StatusAndDescription.h"
#include "xiaohu_robot/TaskStateControlRequest.h"
#include "xiaohu_robot/TaskStateControlResult.h"
#include <sound_play/sound_play.h>
#include <tf/transform_listener.h>

namespace xiaohu_robot {
inline namespace Foundation {
inline namespace Typedefs {
using EmptyMessage = std_msgs::Empty;
using StringMessage = std_msgs::String;
using Float64Message = std_msgs::Float64;
using VelocityCommandMessage = geometry_msgs::Twist;
using CoordinateMessage = geometry_msgs::Pose;
using CoordinateCovarianceStampedMessage = geometry_msgs::PoseWithCovarianceStamped;
using JoystickMessage = sensor_msgs::Joy;
using ManipulatorControlMessage = sensor_msgs::JointState;
using ObjectDetectionResultMessasge = wpb_home_behaviors::Coord;
using TaskStateControlRequestMessage = TaskStateControlRequest;
using InspectionTaskRequestMessage = InspectionTaskRequest;
using MappingTaskRequestMessage = MappingTaskRequest;
using MedicineDeliveryTaskRequestMessage = MedicineDeliveryTaskRequest;
using LegacyGeneralTaskRequestMessage = LegacyGeneralTaskRequest;
using TaskStateControlResultMessage = TaskStateControlResult;
using InspectionTaskResultMessage = InspectionTaskResult;
using MappingTaskResultMessage = MappingTaskResult;
using MedicineDeliveryTaskResultMessage = MedicineDeliveryTaskResult;
using LegacyGeneralTaskResultMessage = LegacyGeneralTaskResult;
using StatusAndDescriptionMessage = StatusAndDescription;

using NodeHandle = ros::NodeHandle;
using Publisher = ros::Publisher;
using Subscriber = ros::Subscriber;
using LoopRate = ros::Rate;

using Length = Measurement<UnitLength>;
using Duration = Measurement<UnitDuration>;
using Frequency = Measurement<UnitFrequency>;
using LinearSpeed = Measurement<UnitSpeed>;
using AngularSpeed = Measurement<UnitAngularSpeed>;
using Temperature = Measurement<UnitTemperature>;

using NavigationClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using SoundplayClient = sound_play::SoundClient;
using ServiceClient = ros::ServiceClient;
using Procedure = std_srvs::Empty;
using TransformListener = tf::TransformListener;

using NavigationGoal = move_base_msgs::MoveBaseGoal;
using GoalState = actionlib::SimpleClientGoalState;
}  // namespace Typedefs
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif
