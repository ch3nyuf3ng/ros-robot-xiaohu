#include "xiaohu_robot/Nodes/JoystickVelocityControlNode.hpp"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/VelocityCommand.hpp"
#include <sstream>
#include <utility>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    ros::init(argc, argv, "joy_vel_ctrl_node");
    JoystickVelocityControlNode::Config joystickVelocityControlConfig{
        CommonConfigs::nodeNamespace,
        CommonConfigs::velocityCommandTopic,
        CommonConfigs::joystickTopic,
        CommonConfigs::messageBufferSize,
        0.2_m_per_s,
        0.5_deg_per_s,
        CommonConfigs::stateCheckingFrequency
    };
    JoystickVelocityControlNode joystickVelocityControlNode{joystickVelocityControlConfig};
    joystickVelocityControlNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
JoystickVelocityControlNode::Config::Config(
    std::string nodeNamespace,
    std::string velocityCommandTopic,
    std::string joystickTopic,
    std::size_t queueSize,
    LinearSpeed maxLinearSpeed,
    AngularSpeed maxAngularSpeed,
    Frequency checkJoystickFrequency
):
    nodeNamespace{std::move(nodeNamespace)},
    velocityCommandTopic{std::move(velocityCommandTopic)},
    joystickTopic{std::move(joystickTopic)},
    queueSize{queueSize},
    maxLinearSpeed{std::move(maxLinearSpeed)},
    maxAngularSpeed{std::move(maxAngularSpeed)},
    checkJoystickFrequency{std::move(checkJoystickFrequency)} {}

std::string JoystickVelocityControlNode::Config::toString() const {
    std::ostringstream oss;
    oss << "JoystickVelocityControl Config:\n"
        << "nodeNamespace: " << nodeNamespace << "\n"
        << "velocityCommandTopic: " << velocityCommandTopic << "\n"
        << "joystickTopic: " << joystickTopic << "\n"
        << "queueSize: " << queueSize << "\n"
        << "maxLinearSpeed" << maxLinearSpeed << "\n"
        << "maxAngularSpeed" << maxAngularSpeed << "\n";

    return oss.str();
}

JoystickVelocityControlNode::JoystickVelocityControlNode(Config config):
    config{std::move(config)},
    nodeState{NodeState::idle},
    nodeHandle{this->config.nodeNamespace},
    currentTiming{0_s},
    velocityCommandMessagePublisher{
        nodeHandle.advertise<VelocityCommandMessage>(this->config.velocityCommandTopic, this->config.queueSize)
    },
    joystickMessageSubscriber{nodeHandle.subscribe<sensor_msgs::Joy>(
        this->config.joystickTopic, this->config.queueSize, &JoystickVelocityControlNode::onReceiveJoystickMessage, this
    )} {
    ROS_INFO("[JoystickVelocityControlNode] Initialized:\n%s", config.toString().c_str());
}

void JoystickVelocityControlNode::sendVelocityCommand(VelocityCommand command) {
    velocityCommandMessagePublisher.publish(command.toMessage());
    ROS_INFO("Velocity command sent: %s", command.toString().c_str());
}

void JoystickVelocityControlNode::onReceiveJoystickMessage(JoystickMessagePointer message) {
    sendVelocityCommand({message, config.maxLinearSpeed, config.maxAngularSpeed});
}

Duration JoystickVelocityControlNode::getTiming() const {
    return currentTiming;
}

void JoystickVelocityControlNode::setTiming(Duration newTiming) {
    currentTiming = newTiming;
}

Frequency JoystickVelocityControlNode::getCheckStateFrequency() const {
    return config.checkJoystickFrequency;
}

NodeState JoystickVelocityControlNode::getNodeState() const {
    return nodeState;
}

void JoystickVelocityControlNode::setNodeState(NodeState newState) {
    nodeState = newState;
    resetTiming();
}

void JoystickVelocityControlNode::whenIsRunning() {
    incrementTiming();
}

void JoystickVelocityControlNode::whenIsIdle() {
    if (getTiming() == 0_s)
        sendVelocityCommand({0_m_per_s, 0_m_per_s, 0_deg_per_s});
    incrementTiming();
}
}  // namespace Nodes
}  // namespace xiaohu_robot