#include "xiaohu_robot/Nodes/MapLoadingModeControllerNode.hpp"
#include "ros/init.h"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <clocale>
#include <cstdlib>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");
    ros::init(argc, argv, CommonConfigs::MapLoadingModeControllerNodeName);
    MapLoadingModeControllerNode mapLoadingModeControllerNode({});
    mapLoadingModeControllerNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
MapLoadingModeControllerNode::MapLoadingModeControllerNode(MapLoadingModeControllerNode::Configs configs):
    nodeHandle(configs.nodeBasicConfigs.nodeNamespace),
    enableMapLoadingModeResultPublisher(nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.enableMapLoadingModeResultTopic,
        configs.nodeBasicConfigs.messageBufferSize
    )),
    mapLoadingRequestSubscriber(nodeHandle.subscribe<MapDataMessage>(
        configs.mapLoadingRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &MapLoadingModeControllerNode::whenReceivedMapLoadingRequest,
        this
    )),
    mapLoadingResultPublisher(nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.mapLoadingResultTopic, configs.nodeBasicConfigs.messageBufferSize
    )),
    mapDataPublisher(
        nodeHandle.advertise<MapDataMessage>(configs.mapDataTopic, configs.nodeBasicConfigs.messageBufferSize, true)
    ),
    configs(std::move(configs)) {
    StatusAndDescriptionMessage result;
    result.status = StatusAndDescriptionMessage::done;
    enableMapLoadingModeResultPublisher.publish(result);
    std::cout << "地图加载器节点已启动。" << std::endl;
}

MapLoadingModeControllerNode::~MapLoadingModeControllerNode() {
    std::cout << "地图加载器节点已退出。" << std::endl;
}

void MapLoadingModeControllerNode::run() {
    ros::spin();
}

void MapLoadingModeControllerNode::whenReceivedMapLoadingRequest(MapDataMessage::ConstPtr const& mapData) {
    mapDataPublisher.publish(mapData);
    StatusAndDescriptionMessage result;
    std::string const command{"cd " + configs.mapSavingPath + " && rosrun map_server map_saver -f map"};
    if (int error{std::system(command.c_str())}) {
        result.status = StatusAndDescriptionMessage::failed;
        result.description = "地图加载失败。";
        std::cerr << result.description << std::endl;
    } else {
        result.status = StatusAndDescriptionMessage::done;
        result.description = "地图加载成功。";
        std::cout << result.description << std::endl;
    }
    mapLoadingResultPublisher.publish(result);
}
}  // namespace Nodes
}  // namespace xiaohu_robot