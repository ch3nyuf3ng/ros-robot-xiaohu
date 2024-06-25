#include "xiaohu_robot/Nodes/ModeSwitchNode.hpp"
#include "ros/init.h"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Exceptions.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <clocale>
#include <cstdlib>
#include <string>
#include <termios.h>
#include <thread>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");
    ros::init(argc, argv, CommonConfigs::ModeSwitchNodeName);
    ModeSwitchNode modeSwitchNode{{ModeSwitchNode::Mode::MapLoading}};
    modeSwitchNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
ModeSwitchNode::ModeSwitchNode(Configs configs):
    mode{configs.initMode},
    nodeHandle(configs.nodeBasicConfigs.nodeNamespace),
    enableServiceModeRequestSubscriber{nodeHandle.subscribe<EmptyMessage>(
        configs.enableServiceModeRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &ModeSwitchNode::whenReceivedEnableServiceModeRequest,
        this
    )},
    enableMappingModeRequestSubscriber{nodeHandle.subscribe<EmptyMessage>(
        configs.enableMappingModeRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &ModeSwitchNode::whenReceivedEnableMappingModeRequest,
        this
    )},
    enableMapLoadingModeRequestSubscriber{nodeHandle.subscribe<EmptyMessage>(
        configs.enableMapLoadingModeRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &ModeSwitchNode::whenReceivedEnableMapLoadingModeRequest,
        this
    )},
    modeNodesThread{},
    configs{std::move(configs)},
    isReal{false} {
    if (nodeHandle.getParam(configs.isRealParameter, isReal) && isReal) {
        std::cout << "实机模式" << std::endl;
    } else {
        std::cout << "模拟模式" << std::endl;
    }
    std::cout << "模式切换节点已启动。" << std::endl;
    runMode(mode);
}

ModeSwitchNode::~ModeSwitchNode() {
    endMode(mode);
    std::cout << "模式切换节点已析构。" << std::endl;
}

void ModeSwitchNode::run() {
    ros::spin();
}

void ModeSwitchNode::endModeExcluding(Mode mode) {
    if (this->mode == mode) {
        return;
    }
    endMode(this->mode);
}

void ModeSwitchNode::whenReceivedEnableModeRequest(Mode mode) {
    endModeExcluding(mode);
    runMode(mode);
}

void ModeSwitchNode::whenReceivedEnableServiceModeRequest(EmptyMessage::ConstPtr const&) {
    whenReceivedEnableModeRequest(Mode::Service);
}

void ModeSwitchNode::whenReceivedEnableMappingModeRequest(EmptyMessage::ConstPtr const&) {
    whenReceivedEnableModeRequest(Mode::Mapping);
}

void ModeSwitchNode::whenReceivedEnableMapLoadingModeRequest(EmptyMessage::ConstPtr const&) {
    whenReceivedEnableModeRequest(Mode::MapLoading);
}

void ModeSwitchNode::runMode(Mode mode) {
    this->mode = mode;
    modeNodesThread = std::thread{[this]() {
        std::string command{
            "roslaunch " + configs.nodeBasicConfigs.nodeNamespace + " " + toString(this->mode) + ".launch"
        };
        if (int error{std::system(command.c_str())}) {
            std::cout << toString(this->mode) << " 退出，返回值：" << error << std::endl;
        }
    }};
}

void ModeSwitchNode::endMode(ModeSwitchNode::Mode mode) {
    std::string command{"rosnode kill "};
    std::string modeChineseName{};
    switch (mode) {
    case Mode::Service:
        command += configs.serviceModeControllerNodeName;
        modeChineseName = "服务";
        break;
    case Mode::Mapping:
        command += configs.mappingModeControllerNodeName;
        modeChineseName = "建图";
        break;
    case Mode::MapLoading:
        command += configs.mapLoadingModeControllerNodeName;
        modeChineseName = "地图加载";
        break;
    }
    if (int error{std::system(command.c_str())}) {
        printMessageThenThrowRuntimeError("结束建图模式失败");
    }
    modeNodesThread.join();
}

std::string ModeSwitchNode::toString(ModeSwitchNode::Mode mode) {
    switch (mode) {
    case Mode::Service:
        return "service_mode";
    case Mode::Mapping:
        return "mapping_mode";
    case Mode::MapLoading:
        return "map_loading_mode";
    }
}
}  // namespace Nodes
}  // namespace xiaohu_robot