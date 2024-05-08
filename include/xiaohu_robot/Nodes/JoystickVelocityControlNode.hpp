#pragma once

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#ifndef XIAOHU_ROBOT_JOYSTICK_HPP
#define XIAOHU_ROBOT_JOYSTICK_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/VelocityCommand.hpp"
#include <string>

namespace xiaohu_robot {
inline namespace Nodes {
struct JoystickVelocityControlNode final: public Runnable {
    struct Config final: public Printable {
        NodeBasicConfig nodeBasicConfig;
        std::string velocityCommandTopic;
        std::string joystickTopic;
        LinearSpeed maxLinearSpeed;
        AngularSpeed maxAngularSpeed;

        Config(
            LinearSpeed maxLinearSpeed,
            AngularSpeed maxAngularSpeed,
            std::string velocityCommandTopic = CommonConfigs::velocityCommandTopic,
            std::string joystickTopic = CommonConfigs::joystickTopic,
            NodeBasicConfig nodeBasicConfig = NodeBasicConfig{}
        );
        std::string toString() const override;
    };

    JoystickVelocityControlNode(Config);
    void run() override;

private:
    Config const config;
    NodeHandle nodeHandle;
    MessagePublisher const velocityCommandMessagePublisher;
    MessageSubscriber const joystickMessageSubscriber;

    void sendVelocityCommand(VelocityCommand);
    void whenReceivedJoystickMessage(JoystickMessagePointer);
};
}  // namespace Nodes
}  // namespace xiaohu_robot
#endif