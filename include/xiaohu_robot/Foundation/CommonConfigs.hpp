#pragma once

#ifndef XIAOHU_ROBOT_CONFIGS_HPP
#define XIAOHU_ROBOT_CONFIGS_HPP

#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <cstddef>

namespace xiaohu_robot {
inline namespace Foundation {
struct CommonConfigs final {
    static std::string const baseStationName;
    static std::string const xiaohuRobotNamespace;
    static std::string const medicineDeliveryTaskRequestTopic;
    static std::string const legacyGeneralTasksRequestTopic;
    static std::string const inspectionTaskRequestTopic;
    static std::string const mappingTaskRequestTopic;
    static std::string const initPositionRequestTopic;
    static std::string const initPositionResultTopic;
    static std::string const joystickResultTopic;
    static std::string const amclInitPositionRequestTopic;
    static std::string const coordinateNavigationTopic;
    static std::string const clearCostmapsTopic;
    static std::string const velocityControlRequestTopic;
    static std::string const manipulatorControlRequestTopic;
    static std::string const medcineDetectionRequestTopic;
    static std::string const medicineDetectionResultTopic;
    static std::string const waypointNavigationRequestTopic;
    static std::string const waypointNavigationResultTopic;
    static std::string const medicineGraspRequestTopic;
    static std::string const medicineGraspResultTopic;
    static std::string const textToSpeechRequestTopic;
    static std::string const textToSpeechResultTopic;
    static std::string const speechRecognitionRequestTopic;
    static std::string const speechRecognitionResultTopic;
    static std::string const xfyunApiLoginParams;
    static std::size_t const messageBufferSize;
    static std::string const& packagePath();
    static Frequency const& loopFrequency();
    static Duration const& maxTextSpeakingDuration();
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif