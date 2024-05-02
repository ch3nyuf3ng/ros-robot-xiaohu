#pragma once

#ifndef XIAOHU_ROBOT_CONFIGS_HPP
#define XIAOHU_ROBOT_CONFIGS_HPP

#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <cstddef>

namespace xiaohu_robot {
struct CommonConfigs {
    static char const* const baseStationName;
    static char const* const velocityCommandTopic;
    static char const* const joystickTopic;
    static char const* const objectDetectionControlTopic;
    static char const* const navigationWaypointTopic;
    static char const* const grabObjectCoodinateTopic;
    static char const* const manipulatorControlTopic;
    static char const* const detectedObjectCoordinatesTopic;
    static char const* const navigationResultTopic;
    static char const* const objectGrabbingResultTopic;
    static char const* const speakTextTopic;

    static char const* const nodeNamespace;
    static char const* const objectMovingTasksTopic;
    static char const* const taskStateControlTopic;

    static std::size_t const messageBufferSize;
    static Frequency const stateCheckingFrequency;
};
}  // namespace xiaohu_robot

#endif