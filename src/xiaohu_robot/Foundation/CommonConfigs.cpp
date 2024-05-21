#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "ros/package.h"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
std::string const CommonConfigs::baseStationName{"base"};
std::string const CommonConfigs::xiaohuRobotNamespace{"xiaohu_robot"};
std::string const CommonConfigs::legacyGeneralTasksRequestTopic{"legacy_tasks_request"};
std::string const CommonConfigs::inspectionTaskRequestTopic{"inspection_task_request"};
std::string const CommonConfigs::mappingTaskRequestTopic{"mapping_task_request"};
std::string const CommonConfigs::medicineDeliveryTaskRequestTopic{"medicine_delivery_task_request"};
std::string const CommonConfigs::initPositionRequestTopic{"init_position_request"};
std::string const CommonConfigs::initPositionResultTopic{"init_position_result"};
std::string const CommonConfigs::joystickResultTopic{"/joy"};
std::string const CommonConfigs::amclInitPositionRequestTopic{"/initialpose"};
std::string const CommonConfigs::coordinateNavigationTopic{"/move_base"};
std::string const CommonConfigs::clearCostmapsTopic{coordinateNavigationTopic + "/clear_costmaps"};
std::string const CommonConfigs::velocityControlRequestTopic{"/cmd_vel"};
std::string const CommonConfigs::manipulatorControlRequestTopic{"/wpb_home/mani_ctrl"};
std::string const CommonConfigs::medcineDetectionRequestTopic{"/wpb_home/behaviors"};
std::string const CommonConfigs::medicineDetectionResultTopic{"/wpb_home/objects_3d"};
std::string const CommonConfigs::waypointNavigationRequestTopic{"/waterplus/navi_waypoint"};
std::string const CommonConfigs::waypointNavigationResultTopic{"/waterplus/navi_result"};
std::string const CommonConfigs::medicineGraspRequestTopic{"/wpb_home/grab_action"};
std::string const CommonConfigs::medicineGraspResultTopic{"/wpb_home/grab_result"};
std::string const CommonConfigs::textToSpeechRequestTopic{"text_to_speech_request"};
std::string const CommonConfigs::textToSpeechResultTopic{"text_to_speech_result"};
std::string const CommonConfigs::speechRecognitionRequestTopic{"speech_recognition_request"};
std::string const CommonConfigs::speechRecognitionResultTopic{"speech_recognition_result"};

std::string const CommonConfigs::xfyunApiLoginParams{"appid = f897d917"};

std::size_t const CommonConfigs::messageBufferSize{10};

std::string const& CommonConfigs::packagePath() {
    static std::string value{ros::package::getPath("xiaohu_robot")};
    return value;
}

Frequency const& CommonConfigs::loopFrequency() {
    static Frequency value{10_Hz};
    return value;
}

Duration const& CommonConfigs::maxTextSpeakingDuration() {
    static Duration value{10_s};
    return value;
}
}  // namespace Foundation
}  // namespace xiaohu_robot