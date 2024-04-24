#include "ros/init.h"
#include "xiaohu_robot/Controller.hpp"
#include "xiaohu_robot/Configs.hpp"

using namespace xiaohu_robot;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "xiaohu_robot");
    xiaohu_robot::Controller controller{
        Configs::baseStationName,
        Configs::velocityControlTopic,
        Configs::behaviorControlTopic,
        Configs::navigationWaypointNameTopic,
        Configs::objectToGrabCoodinateTopic,
        Configs::manipulatorControlTopic,
        Configs::detectedObjectCoordinatesTopic,
        Configs::navigationResultTopic,
        Configs::objectGrabbingResultTopic,
        Configs::objectMovingTasksTopic,
        Configs::taskStateControlTopic,
        Configs::messageBufferSize,
        Configs::stateCheckingFrequency,
        Configs::initialPositionCalibrationTime
    };
    controller.run();
    return 0;
}
