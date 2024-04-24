#pragma once

#ifndef XIAOHU_ROBOT_CONFIGS_HPP
#define XIAOHU_ROBOT_CONFIGS_HPP

#include "xiaohu_robot/Foundation/Measurement.hpp"

namespace xiaohu_robot {
namespace Configs {
auto constexpr baseStationName{"base"};
auto constexpr velocityControlTopic{"/cmd_vel"};
auto constexpr behaviorControlTopic{"/wpb_home/behaviors"};
auto constexpr navigationWaypointNameTopic{"/waterplus/navi_waypoint"};
auto constexpr objectToGrabCoodinateTopic{"/wpb_home/grab_action"};
auto constexpr manipulatorControlTopic{"/wpb_home/mani_ctrl"};
auto constexpr detectedObjectCoordinatesTopic{"/wpb_home/objects_3d"};
auto constexpr navigationResultTopic{"/waterplus/navi_result"};
auto constexpr objectGrabbingResultTopic{"/wpb_home/grab_result"};
auto constexpr objectMovingTasksTopic{"/xiaohu_robot/ObjectMovingTasks"};
auto constexpr taskStateControlTopic{"/xiaohu_robot/TaskStateControl"};
auto constexpr messageBufferSize{1000};
auto const stateCheckingFrequency{120_Hz};
auto const initialPositionCalibrationTime{10_s};
}  // namespace Configs
}  // namespace xiaohu_robot

#endif