#pragma once

#ifndef XIAOHU_ROBOT_NODES_GMAPPING_CONTROLLER_NODE
#define XIAOHU_ROBOT_NODES_GMAPPING_CONTROLLER_NODE

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Empty.h>
#include <tf/transform_listener.h>
#include <thread>

namespace xiaohu_robot {
inline namespace Nodes {
class MapServiceNode final: public Runnable {
public:
    struct Configs final {
        std::string gmappingStartRequestTopic{CommonConfigs::gmappingStartRequestTopic};
        std::string gmappingStartResultTopic{CommonConfigs::gmappingStartResultTopic};
        std::string gmappingStopRequestTopic{CommonConfigs::gmappingStopRequestTopic};
        std::string gmappingStopResultTopic{CommonConfigs::gmappingStopResultTopic};
        std::string gmappingGetMapTopic{CommonConfigs::gmappingGetMapTopic};
        std::string mapSavingRequestTopic{CommonConfigs::mapSavingRequestTopic};
        std::string mapSavingResultTopic{CommonConfigs::mapSavingResultTopic};
        std::string globalMapTopic{CommonConfigs::globalMapTopic};
        std::string baseLinkTopic{CommonConfigs::baseLinkTopic};
        std::string currentPositionRequestTopic{CommonConfigs::currentPositionRequestTopic};
        std::string currentPositionResultTopic{CommonConfigs::currentPositionResultTopic};
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
    Publisher gmappingStartResultPublisher;
    Subscriber gmappingStopRequestSubscriber;
    Publisher gmappingStopResultPublisher;
    Subscriber mapSavingRequestSubscriber;
    Publisher mapSavingResultPublisher;
    Subscriber currentPositionRequestSubscriber;
    Publisher currentPositionResultPublisher;

    Publisher mapPublisher;
    ServiceClient getMapServiceClient;
    TransformListener transformListener;
    bool isGmappingRunning;
    std::thread gmappingThread;
    Configs configs;

    void whenReceivedGmappingStartRequest(EmptyMessage::ConstPtr const&);
    void whenReceivedGmappingStopRequest(EmptyMessage::ConstPtr const&);
    void whenReceivedCurrentPositionRequest(std_msgs::Empty::ConstPtr const&);
    void whenReceivedMapSavingRequest(nav_msgs::OccupancyGrid::ConstPtr const&);
    void startGMapping();
    void stopGMapping();
    void startGMappingThread();
    void publishEmptyMap();
    void saveCurrentMap();
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif