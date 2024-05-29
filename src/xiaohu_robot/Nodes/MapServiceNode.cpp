#include "xiaohu_robot/Nodes/MapServiceNode.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <clocale>
#include <cstdlib>
#include <iostream>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <stdexcept>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    setlocale(LC_ALL, "zh_CN.utf8");
    ros::init(argc, argv, "gmapping_controller_node");
    MapServiceNode gmappingControllerNode{{}};
    gmappingControllerNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
MapServiceNode::MapServiceNode(Configs configs):
    nodeHandle{configs.nodeBasicConfigs.nodeNamespace},
    gmappingStartRequestSubscriber{nodeHandle.subscribe<EmptyMessage>(
        configs.gmappingStartRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &MapServiceNode::whenReceivedGmappingStartRequest,
        this
    )},
    gmappingStartResultPublisher{nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.gmappingStartResultTopic, configs.nodeBasicConfigs.messageBufferSize
    )},
    gmappingStopRequestSubscriber{nodeHandle.subscribe<EmptyMessage>(
        configs.gmappingStopRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &MapServiceNode::whenReceivedGmappingStopRequest,
        this
    )},
    gmappingStopResultPublisher{nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.gmappingStopResultTopic, configs.nodeBasicConfigs.messageBufferSize
    )},
    mapSavingRequestSubscriber{nodeHandle.subscribe<nav_msgs::OccupancyGrid>(
        configs.mapSavingRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &MapServiceNode::whenReceivedMapSavingRequest,
        this
    )},
    mapSavingResultPublisher{nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.mapSavingResultTopic, configs.nodeBasicConfigs.messageBufferSize
    )},
    mapPublisher{nodeHandle.advertise<nav_msgs::OccupancyGrid>(
        configs.globalMapTopic, configs.nodeBasicConfigs.messageBufferSize
    )},

    transformListener{},
    currentPositionResultPublisher{nodeHandle.advertise<geometry_msgs::Pose>("current_position_result", 10)},
    currentPositionRequestSubscriber{
        nodeHandle.subscribe("current_position_request", 10, &MapServiceNode::whenReceivedCurrentPositionRequest, this)
    },
    isGmappingRunning{false},
    gmappingThread{},
    getMapServiceClient{nodeHandle.serviceClient<nav_msgs::GetMap>(configs.gmappingGetMapTopic)},
    configs{std::move(configs)} {
    std::cout << "建图节点已启动。" << std::endl;
}

MapServiceNode::~MapServiceNode() {
    if (isGmappingRunning) {
        stopGMapping();
        isGmappingRunning = false;
        std::cout << "GMapping 已停止。" << std::endl;
    }
    if (gmappingThread.joinable()) {
        gmappingThread.join();
    }
    std::cout << "建图节点已退出。" << std::endl;
}

void MapServiceNode::run() {
    ros::spin();
}

void MapServiceNode::whenReceivedGmappingStartRequest(EmptyMessage::ConstPtr const& msg) {
    if (!isGmappingRunning) {
        publishEmptyMap();
        startGMapping();
        isGmappingRunning = true;
        ROS_INFO("GMapping 已启动。");
        StatusAndDescriptionMessage result;
        result.status = StatusAndDescriptionMessage::done;
        gmappingStartResultPublisher.publish(result);
    } else {
        ROS_WARN("GMapping 已经运行了。");
        StatusAndDescriptionMessage result;
        result.status = StatusAndDescriptionMessage::cancelled;
        gmappingStartResultPublisher.publish(result);
    }
}

void MapServiceNode::whenReceivedGmappingStopRequest(EmptyMessage::ConstPtr const& msg) {
    if (isGmappingRunning) {
        saveCurrentMap();
        stopGMapping();
        isGmappingRunning = false;
        ROS_INFO("GMapping 已停止。");
        StatusAndDescriptionMessage result;
        result.status = StatusAndDescriptionMessage::done;
        gmappingStartResultPublisher.publish(result);
    } else {
        ROS_WARN("GMapping 不在运行中。");
        StatusAndDescriptionMessage result;
        result.status = StatusAndDescriptionMessage::cancelled;
        gmappingStartResultPublisher.publish(result);
    }
}

void MapServiceNode::whenReceivedCurrentPositionRequest(std_msgs::Empty::ConstPtr const&) {
    tf::StampedTransform transform;
    try {
        transformListener.lookupTransform(configs.globalMapTopic, configs.baseLinkTopic, ros::Time(0), transform);
    } catch (tf::TransformException const& ex) {
        ROS_ERROR("获取当前坐标失败：%s", ex.what());
        return;
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

void MapServiceNode::whenReceivedMapSavingRequest(nav_msgs::OccupancyGrid::ConstPtr const& request) {
    mapPublisher.publish(request);
    saveCurrentMap();
    StatusAndDescriptionMessage result;
    result.status = StatusAndDescriptionMessage::done;
    gmappingStartResultPublisher.publish(result);
}

void MapServiceNode::startGMapping() {
    gmappingThread = std::thread(&MapServiceNode::startGMappingThread, this);
}

void MapServiceNode::startGMappingThread() {
    int error{std::system("rosrun gmapping slam_gmapping &")};
    if (error) {
        ROS_WARN("rosrun gmapping slam_gmapping & 返回值：%d", error);
    }
}

void MapServiceNode::stopGMapping() {
    int error{std::system("rosnode kill slam_gmapping")};
    if (error) {
        ROS_WARN("rosnode kill gmapping 返回值：%d", error);
    }
    if (gmappingThread.joinable()) {
        gmappingThread.join();
    }
}

void MapServiceNode::publishEmptyMap() {
    nav_msgs::OccupancyGrid emptyMap;
    emptyMap.header.frame_id = "map";
    emptyMap.info.resolution = 1.0;
    emptyMap.info.width = 0;
    emptyMap.info.height = 0;
    emptyMap.data.clear();

    mapPublisher.publish(emptyMap);
    ROS_INFO("发布了空白地图。");
}

void MapServiceNode::saveCurrentMap() {
    std::ostringstream command;
    command << "cd " << CommonConfigs::packagePath() << "/maps/runtime/ && ";
    command << "rosrun map_server map_saver -f map";
    int error{std::system(command.str().c_str())};
    if (error) {
        ROS_WARN("%s 返回值：%d", command.str().c_str(), error);
        throw std::runtime_error("保存当前地图失败。");
    }
}
}  // namespace Nodes
}  // namespace xiaohu_robot