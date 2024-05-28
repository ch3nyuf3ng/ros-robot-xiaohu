#pragma once

#ifndef XIAOHU_ROBOT_NODES_GMAPPING_CONTROLLER_NODE
#define XIAOHU_ROBOT_NODES_GMAPPING_CONTROLLER_NODE

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>
#include <thread>

namespace xiaohu_robot {
inline namespace Nodes {
class MapServiceNode final: public Runnable {
public:
    struct Configs final {
        std::string gmappingStartRequestTopic{CommonConfigs::gmappingStartRequestTopic};
        std::string gmappingStopRequestTopic{CommonConfigs::gmappingStopRequestTopic};
        std::string gmappingGetMapTopic{CommonConfigs::gmappingGetMapTopic};
        std::string mapSavingRequestTopic{CommonConfigs::mapSavingRequestTopic};
        std::string globalMapTopic{CommonConfigs::globalMapTopic};
        NodeBasicConfigs nodeBasicConfigs{};
    };

    MapServiceNode(Configs);
    MapServiceNode(MapServiceNode const&) = delete;
    MapServiceNode(MapServiceNode&&) = delete;
    MapServiceNode& operator=(MapServiceNode const&) = delete;
    MapServiceNode& operator=(MapServiceNode&&) = delete;
    ~MapServiceNode();
    void run() override;

private:
    NodeHandle nodeHandle;
    Subscriber gmappingStartRequestSubscriber;
    Subscriber gmappingStopRequestSubscriber;
    Subscriber mapSavingRequestSubscriber;
    Publisher mapPublisher;
    ServiceClient getMapServiceClient;
    bool isGmappingRunning;
    std::thread gmappingThread;

    void whenReceivedGmappingStartRequest(EmptyMessage::ConstPtr const&);
    void whenReceivedGmappingStopRequest(EmptyMessage::ConstPtr const&);
    void startGMapping();
    void stopGMapping();
    void startGMappingThread();
    void publishEmptyMap();
    void saveCurrentMap();
    void saveMap(nav_msgs::OccupancyGrid::ConstPtr const&);
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif