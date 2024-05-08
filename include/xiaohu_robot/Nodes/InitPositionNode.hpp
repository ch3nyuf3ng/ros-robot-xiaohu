#pragma once

#ifndef XIAOHU_ROBOT_INIT_POSITION_NODE_HPP
#define XIAOHU_ROBOT_INIT_POSITION_NODE_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"

namespace xiaohu_robot {
inline namespace Nodes {
class InitPositionNode final: public Runnable {
public:
    InitPositionNode(
        std::string initPositionWithCoordinateTopic = CommonConfigs::initPositionWithCoordinateTopic,
        std::string initPositionWithCoordinateCovarianceStampedTopic
        = CommonConfigs::initPositionWithCoordinateCovarianceStampedTopic,
        std::string clearCostmapsTopic = CommonConfigs::clearCostmapsTopic,
        NodeBasicConfig const& nodeBasicConfig = NodeBasicConfig{}
    );
    void run() override;

private:
    NodeHandle nodeHandle;
    MessageSubscriber const initPositionWithCoordinateTopicMessageSubscriber;
    MessagePublisher const initPositionWithCoordinateCovarianceStampedMessagePublisher;
    ServiceClient clearCostmapsClient;

    void whenReceiveInitPositionCoordinateMessage(CoordinateMessagePointer);
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif