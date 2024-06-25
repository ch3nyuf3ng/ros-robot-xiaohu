#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_TYPE_HPP
#define XIAOHU_ROBOT_FOUNDATION_TYPE_HPP

#include "actionlib/client/simple_action_client.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Twist.h"
#include "move_base_msgs/MoveBaseAction.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Empty.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "wpb_home_behaviors/Coord.h"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/InspectionTaskRequest.h"
#include "xiaohu_robot/InspectionTaskResult.h"
#include "xiaohu_robot/MedicineDeliveryTaskRequest.h"
#include "xiaohu_robot/MedicineDeliveryTaskResult.h"
#include "xiaohu_robot/StatusAndDescription.h"
#include "xiaohu_robot/TaskStateControlRequest.h"
#include "xiaohu_robot/TaskStateControlResult.h"
#include <sound_play/sound_play.h>
#include <tf/transform_listener.h>

namespace xiaohu_robot {
inline namespace Foundation {
using AngularSpeed = Measurement<UnitAngularSpeed>;
using CoordinateMessage = geometry_msgs::Pose;
using CoordinateCovarianceStampedMessage = geometry_msgs::PoseWithCovarianceStamped;
using Duration = Measurement<UnitDuration>;
using EmptyMessage = std_msgs::Empty;
using Float64Message = std_msgs::Float64;
using Frequency = Measurement<UnitFrequency>;
using GoalState = actionlib::SimpleClientGoalState;
using InspectionTaskRequestMessage = InspectionTaskRequest;
using Length = Measurement<UnitLength>;
using LinearSpeed = Measurement<UnitSpeed>;
using LoopRate = ros::Rate;
using ManipulatorControlMessage = sensor_msgs::JointState;
using MapDataMessage = nav_msgs::OccupancyGrid;
using MedicineDeliveryTaskRequestMessage = MedicineDeliveryTaskRequest;
using MedicineDeliveryTaskResultMessage = MedicineDeliveryTaskResult;
using NavigationClient = actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>;
using NavigationGoal = move_base_msgs::MoveBaseGoal;
using NodeHandle = ros::NodeHandle;
using ObjectDetectionResultMessasge = wpb_home_behaviors::Coord;
using Procedure = std_srvs::Empty;
using Publisher = ros::Publisher;
using TaskStateControlRequestMessage = TaskStateControlRequest;
using TaskStateControlResultMessage = TaskStateControlResult;
using InspectionTaskResultMessage = InspectionTaskResult;
using ServiceClient = ros::ServiceClient;
using SoundplayClient = sound_play::SoundClient;
using StatusAndDescriptionMessage = StatusAndDescription;
using StringMessage = std_msgs::String;
using Subscriber = ros::Subscriber;
using Temperature = Measurement<UnitTemperature>;
using TransformListener = tf::TransformListener;
using VelocityCommandMessage = geometry_msgs::Twist;
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif
