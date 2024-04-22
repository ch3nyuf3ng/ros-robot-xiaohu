#include "ros/init.h"
#include "ros/node_handle.h"
#include "ros/rate.h"
#include "xiaohu/MoveTaskMsg.h"

using xiaohu::MoveTaskMsg;

int main(int argc, char* argv[]) {
    ros::init(argc, argv, "publish_move_tasks");
    ros::NodeHandle n;
    auto publisher{n.advertise<MoveTaskMsg>("/xiaohu/move_tasks", 1000)};
    auto loop_rate{ros::Rate(10)};
    std::cout << "Input:" << std::endl;
    while (ros::ok()) {
        std::string from_place, to_place;
        std::cin >> from_place >> to_place;
        MoveTaskMsg msg;
        msg.storage_place_name = from_place;
        msg.drop_place_name = to_place;
        publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}