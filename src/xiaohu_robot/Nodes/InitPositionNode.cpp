#include "xiaohu_robot/Nodes/InitPositionNode.hpp"
#include "ros/init.h"
#include "ros/time.h"

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;

    ros::init(argc, argv, "init_position_node");
    InitPositionNode initPositionNode{};
    initPositionNode.run();

    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
InitPositionNode::InitPositionNode(
    std::string initPositionWithCoordinateTopic,
    std::string initPositionWithCoordinateCovarianceStampedTopic,
    std::string clearCostmapsTopic,
    NodeBasicConfig const& nodeBasicConfig
):
    nodeHandle{},
    initPositionWithCoordinateTopicMessageSubscriber(nodeHandle.subscribe<CoordinateMessage>(
        std::move(initPositionWithCoordinateTopic),
        nodeBasicConfig.messageBufferSize,
        &InitPositionNode::whenReceiveInitPositionCoordinateMessage,
        this
    )),
    initPositionWithCoordinateCovarianceStampedMessagePublisher(
        nodeHandle.advertise<CoordinateCovarianceStampedMessage>(
            std::move(initPositionWithCoordinateCovarianceStampedTopic), nodeBasicConfig.messageBufferSize
        )
    ),
    clearCostmapsClient(nodeHandle.serviceClient<std_srvs::Empty>(std::move(clearCostmapsTopic))) {
    sleep(1);
}

void InitPositionNode::run() {
    ros::spin();
}

void InitPositionNode::whenReceiveInitPositionCoordinateMessage(CoordinateMessagePointer receivedMessage) {
    CoordinateCovarianceStampedMessage messageToPublish;
    messageToPublish.header.frame_id = "map";
    messageToPublish.header.stamp = ros::Time::now();
    messageToPublish.pose.pose = *receivedMessage;
    initPositionWithCoordinateCovarianceStampedMessagePublisher.publish(messageToPublish);
    Procedure clearCostmapsService{};
    clearCostmapsClient.call(clearCostmapsService);
}
}  // namespace Nodes
}  // namespace xiaohu_robot