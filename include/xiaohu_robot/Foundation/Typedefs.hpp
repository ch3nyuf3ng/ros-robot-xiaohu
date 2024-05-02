#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_TYPE_HPP
#define XIAOHU_ROBOT_FOUNDATION_TYPE_HPP

#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/rate.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/String.h"
#include "wpb_home_behaviors/Coord.h"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/MoveTaskMsg.h"
#include "xiaohu_robot/NodeControlMessage.h"

namespace xiaohu_robot {
inline namespace Foundation {
inline namespace Typedefs{
using StringMessage = std_msgs::String;
using VelocityCommandMessage = geometry_msgs::Twist;
using CoordinateMessage = geometry_msgs::Pose;
using ManipulatorControlMessage = sensor_msgs::JointState;
using ObjectDetectionResultMessasge = wpb_home_behaviors::Coord;

using StringMessagePointer = StringMessage::ConstPtr const&;
using NodeControlMessagePointer = NodeControlMessage::ConstPtr const&;
using ObjectMovingTaskMessagePointer = MoveTaskMsg::ConstPtr const&;
using ObjectDetectionResultMessasgePointer = ObjectDetectionResultMessasge::ConstPtr const&;
using JoystickMessagePointer = sensor_msgs::Joy::ConstPtr const&;
using CoordinateMessagePointer = CoordinateMessage::ConstPtr const&;

using NodeHandle = ros::NodeHandle;
using MessagePublisher = ros::Publisher;
using MessageSubscriber = ros::Subscriber;
using LoopRate = ros::Rate;

using Length = Measurement<UnitLength>;
using Duration = Measurement<UnitDuration>;
using Frequency = Measurement<UnitFrequency>;
using LinearSpeed = Measurement<UnitSpeed>;
using AngularSpeed = Measurement<UnitAngularSpeed>;
}
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif
