#include "xiaohu_robot/Nodes/InitPositionNode.hpp"
#include "ros/init.h"
#include "ros/time.h"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <chrono>
#include <clocale>
#include <thread>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");

    ros::init(argc, argv, "init_position_node");
    InitPositionNode initPositionNode{InitPositionNode::Configs{}};
    initPositionNode.run();

    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
InitPositionNode::InitPositionNode(Configs configs):
    nodeHandle(configs.nodeBasicConfigs.nodeNamespace),
    initPositionRequestSubscriber{nodeHandle.subscribe<CoordinateMessage>(
        configs.initPositionRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &InitPositionNode::whenReceivedInitPositionRequest,
        this
    )},
    initPositionResultPublisher{nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.initPositionResultTopic, configs.nodeBasicConfigs.messageBufferSize
    )},
    amclInitPositionRequestTopicPublisher{nodeHandle.advertise<CoordinateCovarianceStampedMessage>(
        configs.amclInitPositionRequestTopic, configs.nodeBasicConfigs.messageBufferSize
    )} {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "初始位置设置节点已启动。" << std::endl;
}

InitPositionNode::~InitPositionNode() {
    std::cout << "初始位置设置节点已退出。" << std::endl;
}

void InitPositionNode::run() {
    ros::spin();
}

void InitPositionNode::whenReceivedInitPositionRequest(CoordinateMessage::ConstPtr const& receivedMessage) {
    CoordinateCovarianceStampedMessage amclInitPositionRequest;
    amclInitPositionRequest.header.frame_id = "map";
    amclInitPositionRequest.header.stamp = ros::Time::now();
    amclInitPositionRequest.pose.pose = *receivedMessage;
    amclInitPositionRequestTopicPublisher.publish(amclInitPositionRequest);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    StatusAndDescriptionMessage result;
    result.status = StatusAndDescriptionMessage::done;
    initPositionResultPublisher.publish<StatusAndDescriptionMessage>(result);
}
}  // namespace Nodes
}  // namespace xiaohu_robot