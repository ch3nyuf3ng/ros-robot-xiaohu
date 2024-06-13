#pragma once

#ifndef XIAOHU_ROBOT_NODES_MAP_LOADING_NODE_HPP
#define XIAOHU_ROBOT_NODES_MAP_LOADING_NODE_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"

namespace xiaohu_robot {
inline namespace Nodes {
class MapLoadingModeControllerNode final: public Runnable {
public:
    struct Configs final {
        std::string mapDataTopic{CommonConfigs::MapDataTopic};
        std::string mapLoadingRequestTopic{CommonConfigs::MapLoadingRequestTopic};
        std::string mapLoadingResultTopic{CommonConfigs::EnableMapLoadingModeResultTopic};
        std::string mapSavingPath{CommonConfigs::PackagePath() + "/maps/runtime/"};
        NodeBasicConfigs nodeBasicConfigs{};
    };

    MapLoadingModeControllerNode(Configs);
    MapLoadingModeControllerNode(MapLoadingModeControllerNode const&) = delete;
    MapLoadingModeControllerNode(MapLoadingModeControllerNode&&) = delete;
    MapLoadingModeControllerNode& operator=(MapLoadingModeControllerNode const&) = delete;
    MapLoadingModeControllerNode& operator=(MapLoadingModeControllerNode&&) = delete;
    ~MapLoadingModeControllerNode();

    void run() override;

private:
    void whenReceivedMapLoadingRequest(MapDataMessage::ConstPtr const&);

    NodeHandle nodeHandle;
    Subscriber mapLoadingRequestSubscriber;
    Publisher mapLoadingResultPublisher;
    Publisher mapDataPublisher;
    Configs configs;
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif