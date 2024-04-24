#include "xiaohu_robot/Configs.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "std_msgs/String.h"
#include <iostream>

using namespace xiaohu_robot;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "cancel_move_tasks_node");
    ros::NodeHandle node_handle{};
    auto publisher{node_handle.advertise<std_msgs::String>(Configs::taskStateControlTopic, Configs::messageBufferSize)};
    ros::Rate loopRate{Configs::stateCheckingFrequency};
    while (ros::ok()) {
        std::string option;
        std::cin >> option;
        if (option == "cancel") {
            std_msgs::String message{};
            message.data = option;
            publisher.publish(message);
        }
        ros::spinOnce();
        loopRate.sleep();
    }
}