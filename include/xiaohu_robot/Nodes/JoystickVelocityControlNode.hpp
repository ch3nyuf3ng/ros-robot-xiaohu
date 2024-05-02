#pragma once

#ifndef XIAOHU_ROBOT_JOYSTICK_HPP
#define XIAOHU_ROBOT_JOYSTICK_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/VelocityCommand.hpp"
#include <cstddef>
#include <string>

namespace xiaohu_robot {
inline namespace Nodes {
struct JoystickVelocityControlNode final: public EnableNodeControl {
    struct Config: public Printable {
        std::string nodeNamespace;
        std::string nodeControlTopic;
        std::string velocityCommandTopic;
        std::string joystickTopic;
        std::size_t queueSize;
        LinearSpeed maxLinearSpeed;
        AngularSpeed maxAngularSpeed;
        Frequency checkJoystickFrequency;

        Config(
            std::string nodeNamespace,
            std::string velocityCommandTopic,
            std::string joystickTopic,
            std::size_t queueSize,
            LinearSpeed maxLinearSpeed,
            AngularSpeed maxAngularSpeed,
            Frequency checkJoystickFrequency
        );
        std::string toString() const override;
    };

    JoystickVelocityControlNode(Config);

private:
    Config config;
    NodeState nodeState;
    NodeHandle nodeHandle;
    Duration currentTiming;
    MessagePublisher const velocityCommandMessagePublisher;
    MessageSubscriber const joystickMessageSubscriber;

    Duration getTiming() const override;
    Frequency getCheckStateFrequency() const override;
    NodeState getNodeState() const override;
    void setTiming(Duration) override;
    void setNodeState(NodeState) override;
    void whenIsRunning() override;
    void whenIsIdle() override;
    void sendVelocityCommand(VelocityCommand);
    void onReceiveJoystickMessage(JoystickMessagePointer);
};
}  // namespace Nodes
}  // namespace xiaohu_robot
#endif