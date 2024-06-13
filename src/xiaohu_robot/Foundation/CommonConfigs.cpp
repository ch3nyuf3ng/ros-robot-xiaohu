#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "ros/package.h"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
std::string const CommonConfigs::AmclInitPositionRequestTopic{"/initialpose"};
std::string const CommonConfigs::BaseLinkTopic{"/base_footprint"};
std::string const CommonConfigs::ClearCostmapsTopic{"/move_base/clear_costmaps"};
std::string const CommonConfigs::CoordinateNavigationTopic{"/move_base"};
std::string const CommonConfigs::CurrentTaskStateRequestTopic{"current_task_state_request"};
std::string const CommonConfigs::CurrentTaskStateResultTopic{"current_task_state_result"};
std::string const CommonConfigs::CurrentPositionRequestTopic{"current_position_request"};
std::string const CommonConfigs::CurrentPositionResultTopic{"current_position_result"};
std::string const CommonConfigs::EnableMappingModeRequestTopic{"enable_mapping_mode_request"};
std::string const CommonConfigs::EnableMappingModeResultTopic{"enable_mapping_mode_result"};
std::string const CommonConfigs::EnableMapLoadingModeRequestTopic{"enable_map_loading_mode_request"};
std::string const CommonConfigs::EnableMapLoadingModeResultTopic{"enable_map_loading_mode_result"};
std::string const CommonConfigs::EnableServiceModeRequestTopic{"enable_service_mode_request"};
std::string const CommonConfigs::EnableServiceModeResultTopic{"enable_service_mode_result"};
std::string const CommonConfigs::GmappingGetMapTopic{"/dynamic_map"};
std::string const CommonConfigs::GmappingStartRequestTopic{"gmapping_start_request"};
std::string const CommonConfigs::GmappingStartResultTopic{"gmapping_start_result"};
std::string const CommonConfigs::GmappingStopRequestTopic{"gmapping_stop_request"};
std::string const CommonConfigs::GmappingStopResultTopic{"gmapping_stop_result"};
std::string const CommonConfigs::InitPositionRequestTopic{"init_position_request"};
std::string const CommonConfigs::InitPositionResultTopic{"init_position_result"};
std::string const CommonConfigs::InspectionTaskRequestTopic{"inspection_task_request"};
std::string const CommonConfigs::InspectionTaskResultTopic{"inspection_task_result"};
std::string const CommonConfigs::IsRealParameter{"is_real"};
std::string const CommonConfigs::LegacyGeneralTasksRequestTopic{"legacy_tasks_request"};
std::string const CommonConfigs::LegacyGeneralTasksResultTopic{"legacy_tasks_result"};

Frequency const& CommonConfigs::LoopFrequency() {
    static Frequency value{30_Hz};
    return value;
}

std::string const CommonConfigs::ManipulatorControlRequestTopic{"/wpb_home/mani_ctrl"};
std::string const CommonConfigs::MapDataTopic{"/map"};
std::string const CommonConfigs::MapLoadingModeControllerNodeName{"map_loading_mode_controller_node"};
std::string const CommonConfigs::MapLoadingRequestTopic{"map_saving_request"};
std::string const CommonConfigs::MapLoadingResultTopic{"map_saving_result"};
std::string const CommonConfigs::MappingModeControllerNodeName{"mapping_mode_controller_node"};

Duration const& CommonConfigs::MaxTextSpeakingDuration() {
    static Duration value{10_s};
    return value;
}

std::string const CommonConfigs::MedicineDeliveryTaskRequestTopic{"medicine_delivery_task_request"};
std::string const CommonConfigs::MedicineDeliveryTaskResultTopic{"medicine_delivery_task_result"};
std::string const CommonConfigs::MedicineDetectionRequestTopic{"/wpb_home/behaviors"};
std::string const CommonConfigs::MedicineDetectionResultTopic{"/wpb_home/objects_3d"};
std::string const CommonConfigs::MedicineGraspRequestTopic{"/wpb_home/grab_action"};
std::string const CommonConfigs::MedicineGraspResultTopic{"/wpb_home/grab_result"};
std::size_t const CommonConfigs::MessageBufferSize{10};
std::string const CommonConfigs::ModeSwitchNodeName{"mode_switch_node"};

std::string const& CommonConfigs::PackagePath() {
    static std::string value{ros::package::getPath("xiaohu_robot")};
    return value;
}

std::string const CommonConfigs::ServiceModeControllerNodeName{"service_mode_controller_node"};
std::string const CommonConfigs::SpeechRecognitionNodeName{"speech_recognition_node"};
std::string const CommonConfigs::SpeechRecognitionRequestTopic{"speech_recognition_request"};
std::string const CommonConfigs::SpeechRecognitionResultTopic{"speech_recognition_result"};
std::string const CommonConfigs::TemperatureMeasurementNodeName{"temperature_measurement_node"};
std::string const CommonConfigs::TemperatureMeasurementRequestTopic{"temperature_measurement_request"};
std::string const CommonConfigs::TemperatureMeasurementResultTopic{"temperature_measurement_result"};
std::string const CommonConfigs::TextToSpeechNodeName{"text_to_speech_node"};
std::string const CommonConfigs::TextToSpeechRequestTopic{"text_to_speech_request"};
std::string const CommonConfigs::TextToSpeechResultTopic{"text_to_speech_result"};
std::string const CommonConfigs::VideoCallRequestTopic{"video_call_request"};
std::string const CommonConfigs::VideoCallResultTopic{"viedo_call_result"};
std::string const CommonConfigs::VelocityControlRequestTopic{"/cmd_vel"};
std::string const CommonConfigs::XfyunApiLoginParams{"appid = f897d917"};
std::string const CommonConfigs::XiaohuRobotNamespace{"xiaohu_robot"};
}  // namespace Foundation
}  // namespace xiaohu_robot