#pragma once

#ifndef XIAOHU_ROBOT_NODES_GMAPPING_CONTROLLER_NODE
#define XIAOHU_ROBOT_NODES_GMAPPING_CONTROLLER_NODE

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>

namespace xiaohu_robot {
inline namespace Nodes {
class MappingModeControllerNode final: public Runnable {
public:
    struct Configs final {
        std::string baseLinkTopic{CommonConfigs::BaseLinkTopic};
        std::string currentPositionRequestTopic{CommonConfigs::CurrentPositionRequestTopic};
        std::string currentPositionResultTopic{CommonConfigs::CurrentPositionResultTopic};
        std::string enableMappingModeResultTopic{CommonConfigs::EnableMappingModeResultTopic};
        std::string globalMapTopic{CommonConfigs::MapDataTopic};
        std::string gmappingGetMapTopic{CommonConfigs::GmappingGetMapTopic};
        NodeBasicConfigs nodeBasicConfigs{};
    };

    MappingModeControllerNode(Configs);
    MappingModeControllerNode(MappingModeControllerNode const&) = delete;
    MappingModeControllerNode(MappingModeControllerNode&&) = delete;
    MappingModeControllerNode& operator=(MappingModeControllerNode const&) = delete;
    MappingModeControllerNode& operator=(MappingModeControllerNode&&) = delete;
    ~MappingModeControllerNode();

    void run() override;

private:
    NodeHandle nodeHandle;

    Publisher enableMappingModeResultPublisher;
    Subscriber currentPositionRequestSubscriber;
    Publisher currentPositionResultPublisher;
    Subscriber currentMapRequestSubscriber;
    Publisher currentMapResultPublisher;

    ServiceClient getMapServiceClient;
    TransformListener transformListener;
    Configs configs;

    void whenReceivedCurrentPositionRequest(EmptyMessage::ConstPtr const&);
    void whenReceivedCurrentMapRequest(EmptyMessage::ConstPtr const&);
    void saveCurrentMap();
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif