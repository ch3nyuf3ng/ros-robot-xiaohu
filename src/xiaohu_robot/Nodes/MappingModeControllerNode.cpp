#include "xiaohu_robot/Nodes/MappingModeControllerNode.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Exceptions.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <clocale>
#include <cstdlib>
#include <iostream>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");
    ros::init(argc, argv, CommonConfigs::MappingModeControllerNodeName);
    MappingModeControllerNode gmappingControllerNode{{}};
    gmappingControllerNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
MappingModeControllerNode::MappingModeControllerNode(Configs configs):
    nodeHandle{configs.nodeBasicConfigs.nodeNamespace},
    transformListener{},
    enableMappingModeResultPublisher(nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.enableMappingModeResultTopic,
        configs.nodeBasicConfigs.messageBufferSize
    )),
    currentPositionResultPublisher{nodeHandle.advertise<geometry_msgs::Pose>("current_position_result", 10)},
    currentPositionRequestSubscriber{
        nodeHandle.subscribe("current_position_request", 10, &MappingModeControllerNode::whenReceivedCurrentPositionRequest, this)
    },
    getMapServiceClient{nodeHandle.serviceClient<nav_msgs::GetMap>(configs.gmappingGetMapTopic)},
    configs{std::move(configs)} {
    StatusAndDescriptionMessage result;
    result.status = StatusAndDescriptionMessage::done;
    enableMappingModeResultPublisher.publish(result);
    std::cout << "建图模式已启动。" << std::endl;
}

MappingModeControllerNode::~MappingModeControllerNode() {
    saveCurrentMap();
    std::cout << "建图模式已退出。" << std::endl;
}

void MappingModeControllerNode::run() {
    ros::spin();
}

void MappingModeControllerNode::whenReceivedCurrentPositionRequest(std_msgs::Empty::ConstPtr const&) {
    tf::StampedTransform transform;
    try {
        transformListener.lookupTransform(configs.globalMapTopic, configs.baseLinkTopic, ros::Time(0), transform);
    } catch (tf::TransformException const& ex) {
        ROS_ERROR("获取当前坐标失败：%s", ex.what());
    }

    CoordinateMessage currentPositionResult;
    currentPositionResult.position.x = transform.getOrigin().x();
    currentPositionResult.position.y = transform.getOrigin().y();
    currentPositionResult.position.z = transform.getOrigin().z();

    tf::Quaternion q = transform.getRotation();
    currentPositionResult.orientation.x = q.x();
    currentPositionResult.orientation.y = q.y();
    currentPositionResult.orientation.z = q.z();
    currentPositionResult.orientation.w = q.w();

    currentPositionResultPublisher.publish(currentPositionResult);
}

void MappingModeControllerNode::saveCurrentMap() {
    std::ostringstream command;
    command << "cd " << CommonConfigs::PackagePath() << "/maps/runtime/ && ";
    command << "rosrun map_server map_saver -f map";
    if (int error{std::system(command.str().c_str())}) {
        ROS_WARN("%s 返回值：%d", command.str().c_str(), error);
        printMessageThenThrowRuntimeError("保存当前地图失败。");
    }
}
}  // namespace Nodes
}  // namespace xiaohu_robot