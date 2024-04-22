#include "ros/init.h"
#include "xiaohu_robot/move_objects_node/state_machine.hpp"

using namespace xiaohu_robot;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "serve_drinks");
    xiaohu_robot::move_objects_node::StateMachine state_machine{"base"};
    state_machine.run();
}
