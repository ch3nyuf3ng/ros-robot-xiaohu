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
    struct Configs final {
        std::string initPositionRequestTopic{CommonConfigs::InitPositionRequestTopic};
        std::string initPositionResultTopic{CommonConfigs::InitPositionResultTopic};
        std::string amclInitPositionRequestTopic{CommonConfigs::AmclInitPositionRequestTopic};
        NodeBasicConfigs nodeBasicConfigs{};
    };

    InitPositionNode(Configs);
    ~InitPositionNode();
    void run() override;

private:
    NodeHandle nodeHandle;
    Subscriber const initPositionRequestSubscriber;
    Publisher const initPositionResultPublisher;
    Publisher const amclInitPositionRequestTopicPublisher;
    
    Configs configs;

    void whenReceivedInitPositionRequest(CoordinateMessage::ConstPtr const&);
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif