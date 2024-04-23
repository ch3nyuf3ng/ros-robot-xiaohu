#include "XiaohuRobot/Configs.hpp"
#include "XiaohuRobot/MoveTaskMsg.h"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"

using namespace XiaohuRobot;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "publish_move_tasks");
    ros::NodeHandle n;
    auto publisher{n.advertise<MoveTaskMsg>(Configs::objectMovingTasksTopic, Configs::messageBufferSize)};
    auto loopRate{ros::Rate(Configs::stateCheckingFrequency)};
    std::cout << "Input:" << std::endl;
    while (ros::ok()) {
        std::string from_place, to_place;
        std::cin >> from_place >> to_place;
        MoveTaskMsg msg;
        msg.storage_place_name = from_place;
        msg.drop_place_name = to_place;
        publisher.publish(msg);
        ros::spinOnce();
        loopRate.sleep();
    }
    return 0;
}