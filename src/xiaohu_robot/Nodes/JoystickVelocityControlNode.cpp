#include "xiaohu_robot/Nodes/JoystickVelocityControlNode.hpp"
#include "ros/init.h"
#include <sstream>
#include <utility>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;

    ros::init(argc, argv, "joy_vel_ctrl_node");
    JoystickVelocityControlNode joystickVelocityControlNode{JoystickVelocityControlNode::Config{
        0.2_m_per_s,
        0.5_deg_per_s,
    }};
    joystickVelocityControlNode.run();

    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
JoystickVelocityControlNode::Config::Config(
    LinearSpeed maxLinearSpeed,
    AngularSpeed maxAngularSpeed,
    std::string velocityCommandTopic,
    std::string joystickTopic,
    NodeBasicConfigs nodeBasicConfig
):
    nodeBasicConfig{std::move(nodeBasicConfig)},
    velocityCommandTopic{std::move(velocityCommandTopic)},
    joystickTopic{std::move(joystickTopic)},
    maxLinearSpeed{std::move(maxLinearSpeed)},
    maxAngularSpeed{std::move(maxAngularSpeed)} {
    sleep(1);
}

std::string JoystickVelocityControlNode::Config::toString() const {
    std::ostringstream oss;
    oss << "JoystickVelocityControl Config:\n";
    oss << "velocityCommandTopic: " << velocityCommandTopic << "\n";
    oss << "joystickTopic: " << joystickTopic << "\n";
    oss << "maxLinearSpeed" << maxLinearSpeed << "\n";
    oss << "maxAngularSpeed" << maxAngularSpeed << "\n";
    return oss.str();
}

JoystickVelocityControlNode::JoystickVelocityControlNode(Config config):
    config{std::move(config)},
    nodeHandle{},
    velocityCommandMessagePublisher{nodeHandle.advertise<VelocityCommandMessage>(
        this->config.velocityCommandTopic, this->config.nodeBasicConfig.messageBufferSize
    )},
    joystickMessageSubscriber{nodeHandle.subscribe<sensor_msgs::Joy>(
        this->config.joystickTopic,
        this->config.nodeBasicConfig.messageBufferSize,
        &JoystickVelocityControlNode::whenReceivedJoystickMessage,
        this
    )} {
    ROS_INFO("[JoystickVelocityControlNode] Initialized:\n%s", config.toString().c_str());
}

void JoystickVelocityControlNode::run() {
    ros::Rate loopRate(config.nodeBasicConfig.loopFrequency);
    while (ros::ok()) {
        ros::spinOnce();
        loopRate.sleep();
    }
}

void JoystickVelocityControlNode::sendVelocityCommand(VelocityCommand command) {
    velocityCommandMessagePublisher.publish(command.toMessage());
    ROS_INFO("Velocity command sent: %s", command.toString().c_str());
}

void JoystickVelocityControlNode::whenReceivedJoystickMessage(sensor_msgs::Joy::ConstPtr const& message) {
    sendVelocityCommand({message, config.maxLinearSpeed, config.maxAngularSpeed});
}
}  // namespace Nodes
}  // namespace xiaohu_robot