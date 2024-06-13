#pragma once

#ifndef XIAOHU_ROBOT_CONFIGS_HPP
#define XIAOHU_ROBOT_CONFIGS_HPP

#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <cstddef>

namespace xiaohu_robot {
inline namespace Foundation {
struct CommonConfigs final {
    static std::string const AmclInitPositionRequestTopic;
    static std::string const BaseLinkTopic;
    static std::string const ClearCostmapsTopic;
    static std::string const CoordinateNavigationTopic;
    static std::string const CurrentTaskStateRequestTopic;
    static std::string const CurrentTaskStateResultTopic;
    static std::string const CurrentPositionRequestTopic;
    static std::string const CurrentPositionResultTopic;
    static std::string const EnableMapLoadingModeRequestTopic;
    static std::string const EnableMapLoadingModeResultTopic;
    static std::string const EnableMappingModeRequestTopic;
    static std::string const EnableMappingModeResultTopic;
    static std::string const EnableServiceModeRequestTopic;
    static std::string const EnableServiceModeResultTopic;
    static std::string const GmappingGetMapTopic;
    static std::string const GmappingStartRequestTopic;
    static std::string const GmappingStartResultTopic;
    static std::string const GmappingStopRequestTopic;
    static std::string const GmappingStopResultTopic;
    static std::string const InitPositionRequestTopic;
    static std::string const InitPositionResultTopic;
    static std::string const InspectionTaskRequestTopic;
    static std::string const InspectionTaskResultTopic;
    static std::string const IsRealParameter;
    static std::string const LegacyGeneralTasksRequestTopic;
    static std::string const LegacyGeneralTasksResultTopic;
    static Frequency const& LoopFrequency();
    static std::string const ManipulatorControlRequestTopic;
    static std::string const MapDataTopic;
    static std::string const MapLoadingModeControllerNodeName;
    static std::string const MapLoadingRequestTopic;
    static std::string const MapLoadingResultTopic;
    static std::string const MappingModeControllerNodeName;
    static Duration const& MaxTextSpeakingDuration();
    static std::string const MedicineDeliveryTaskRequestTopic;
    static std::string const MedicineDeliveryTaskResultTopic;
    static std::string const MedicineDetectionRequestTopic;
    static std::string const MedicineDetectionResultTopic;
    static std::string const MedicineGraspRequestTopic;
    static std::string const MedicineGraspResultTopic;
    static std::size_t const MessageBufferSize;
    static std::string const ModeSwitchNodeName;
    static std::string const& PackagePath();
    static std::string const ServiceModeControllerNodeName;
    static std::string const SpeechRecognitionNodeName;
    static std::string const SpeechRecognitionRequestTopic;
    static std::string const SpeechRecognitionResultTopic;
    static std::string const TemperatureMeasurementNodeName;
    static std::string const TemperatureMeasurementRequestTopic;
    static std::string const TemperatureMeasurementResultTopic;
    static std::string const TextToSpeechNodeName;
    static std::string const TextToSpeechRequestTopic;
    static std::string const TextToSpeechResultTopic;
    static std::string const VideoCallRequestTopic;
    static std::string const VideoCallResultTopic;
    static std::string const VelocityControlRequestTopic;
    static std::string const XfyunApiLoginParams;
    static std::string const XiaohuRobotNamespace;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif