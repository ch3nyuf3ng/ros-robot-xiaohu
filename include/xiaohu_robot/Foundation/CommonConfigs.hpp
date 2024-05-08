#pragma once

#ifndef XIAOHU_ROBOT_CONFIGS_HPP
#define XIAOHU_ROBOT_CONFIGS_HPP

#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <cstddef>

namespace xiaohu_robot {
struct CommonConfigs {
    static std::string const velocityCommandTopic;
    static std::string const joystickTopic;
    static std::string const objectDetectionControlTopic;
    static std::string const navigationWaypointTopic;
    static std::string const objectGrabbingCoodinateTopic;
    static std::string const manipulatorControlTopic;
    static std::string const detectedObjectCoordinatesTopic;
    static std::string const navigationResultTopic;
    static std::string const objectGrabbingResultTopic;
    static std::string const speakTextTopic;
    static std::string const moveBaseTopic;
    static std::string const initPositionWithCoordinateCovarianceStampedTopic;
    static std::string const clearCostmapsTopic;

    static std::string const nodeNamespace;
    static std::string const legacyGeneralTasksTopic;
    static std::string const initPositionWithCoordinateTopic;
    static std::string const baseStationName;

    static std::size_t const messageBufferSize;
    static Frequency const loopFrequency;
};
}  // namespace xiaohu_robot

#endif