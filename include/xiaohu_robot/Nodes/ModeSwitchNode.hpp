#pragma once

#ifndef XIAOHU_ROBOT_MODE_SWITCH_NODE_HPP
#define XIAOHU_ROBOT_MODE_SWITCH_NODE_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <thread>

namespace xiaohu_robot {
inline namespace Nodes {
class ModeSwitchNode final: public Runnable {
public:
    enum class Mode {
        Service,
        Mapping,
        MapLoading
    };

    struct Configs {
        Mode initMode;
        std::string enableMapLoadingModeRequestTopic{CommonConfigs::EnableMapLoadingModeRequestTopic};
        std::string enableMappingModeRequestTopic{CommonConfigs::EnableMappingModeRequestTopic};
        std::string enableServiceModeRequestTopic{CommonConfigs::EnableServiceModeRequestTopic};
        std::string isRealParameter{CommonConfigs::IsRealParameter};
        std::string mapLoadingModeControllerNodeName{CommonConfigs::MapLoadingModeControllerNodeName};
        std::string mappingModeControllerNodeName{CommonConfigs::MappingModeControllerNodeName};
        std::string serviceModeControllerNodeName{CommonConfigs::ServiceModeControllerNodeName};
        NodeBasicConfigs nodeBasicConfigs{};
    };

    ModeSwitchNode(Configs);
    ModeSwitchNode(ModeSwitchNode const&) = delete;
    ModeSwitchNode(ModeSwitchNode&&) = delete;
    ModeSwitchNode& operator=(ModeSwitchNode const&) = delete;
    ModeSwitchNode& operator=(ModeSwitchNode&&) = delete;
    ~ModeSwitchNode();
    void run() override;

private:
    Mode mode;
    NodeHandle nodeHandle;
    Subscriber enableServiceModeRequestSubscriber;
    Subscriber enableMappingModeRequestSubscriber;
    Subscriber enableMapLoadingModeRequestSubscriber;
    std::thread modeNodesThread;
    Configs configs;
    bool isReal;

    void whenReceivedEnableModeRequest(Mode);
    void whenReceivedEnableServiceModeRequest(EmptyMessage::ConstPtr const&);
    void whenReceivedEnableMappingModeRequest(EmptyMessage::ConstPtr const&);
    void whenReceivedEnableMapLoadingModeRequest(EmptyMessage::ConstPtr const&);
    void runMode(Mode);
    void endMode(Mode);
    void endModeExcluding(Mode);

    static std::string toString(Mode mode);
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif