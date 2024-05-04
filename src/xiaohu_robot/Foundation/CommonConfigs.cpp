#include "xiaohu_robot/Foundation/CommonConfigs.hpp"

namespace xiaohu_robot {
char const* const CommonConfigs::baseStationName{"base"};
char const* const CommonConfigs::velocityCommandTopic{"/cmd_vel"};
char const* const CommonConfigs::joystickTopic{"joy"};
char const* const CommonConfigs::objectDetectionControlTopic{"/wpb_home/behaviors"};
char const* const CommonConfigs::navigationWaypointTopic{"/waterplus/navi_waypoint"};
char const* const CommonConfigs::grabObjectCoodinateTopic{"/wpb_home/grab_action"};
char const* const CommonConfigs::manipulatorControlTopic{"/wpb_home/mani_ctrl"};
char const* const CommonConfigs::detectedObjectCoordinatesTopic{"/wpb_home/objects_3d"};
char const* const CommonConfigs::navigationResultTopic{"/waterplus/navi_result"};
char const* const CommonConfigs::objectGrabbingResultTopic{"/wpb_home/grab_result"};
char const* const CommonConfigs::speakTextTopic{"/xfyun/tts"};
char const* const CommonConfigs::moveBaseTopic{"move_base"};

char const* const CommonConfigs::nodeNamespace{"xiaohu_robot"};
char const* const CommonConfigs::legacyTasksTopic{"/xiaohu_robot/ObjectMovingTasks"};
char const* const CommonConfigs::legacyGeneralTasksTopic{"/xiaohu_robot/general_tasks"};
char const* const CommonConfigs::taskStateControlTopic{"/xiaohu_robot/TaskStateControl"};

std::size_t const CommonConfigs::messageBufferSize{60};
Frequency const CommonConfigs::stateCheckingFrequency{60_Hz};
}  // namespace xiaohu_robot