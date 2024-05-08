#include "xiaohu_robot/Foundation/CommonConfigs.hpp"

namespace xiaohu_robot {
std::string const CommonConfigs::velocityCommandTopic{"/cmd_vel"};
std::string const CommonConfigs::joystickTopic{"joy"};
std::string const CommonConfigs::objectDetectionControlTopic{"/wpb_home/behaviors"};
std::string const CommonConfigs::navigationWaypointTopic{"/waterplus/navi_waypoint"};
std::string const CommonConfigs::objectGrabbingCoodinateTopic{"/wpb_home/grab_action"};
std::string const CommonConfigs::manipulatorControlTopic{"/wpb_home/mani_ctrl"};
std::string const CommonConfigs::detectedObjectCoordinatesTopic{"/wpb_home/objects_3d"};
std::string const CommonConfigs::navigationResultTopic{"/waterplus/navi_result"};
std::string const CommonConfigs::objectGrabbingResultTopic{"/wpb_home/grab_result"};
std::string const CommonConfigs::speakTextTopic{"/xfyun/tts"};
std::string const CommonConfigs::moveBaseTopic{"move_base"};
std::string const CommonConfigs::initPositionWithCoordinateCovarianceStampedTopic{"/initialpose"};
std::string const CommonConfigs::clearCostmapsTopic{"/move_base/clear_costmaps"};

std::string const CommonConfigs::nodeNamespace{"xiaohu_robot"};
std::string const CommonConfigs::legacyGeneralTasksTopic{nodeNamespace + "/general_tasks"};
std::string const CommonConfigs::initPositionWithCoordinateTopic{nodeNamespace + "/init_position"};
std::string const CommonConfigs::baseStationName{"base"};

std::size_t const CommonConfigs::messageBufferSize{60};
Frequency const CommonConfigs::loopFrequency{60_Hz};
}  // namespace xiaohu_robot