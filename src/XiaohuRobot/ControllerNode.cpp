#include "ros/init.h"
#include "XiaohuRobot/Controller.hpp"
#include "XiaohuRobot/Configs.hpp"

using namespace XiaohuRobot;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "XiaohuRobot");
    XiaohuRobot::Controller controller{
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
