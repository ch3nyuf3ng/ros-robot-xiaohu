#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "ros/init.h"
#include "ros/this_node.h"
#include <stdexcept>

using namespace xiaohu_robot;

std::string const NodeControlBehavior::pause{"pause"};
std::string const NodeControlBehavior::resume{"resume"};

void EnableNodeControl::incrementTiming() {
    setTiming(getTiming() + getCheckStateFrequency().perCycleTime());
}

void EnableNodeControl::resetTiming() {
    setTiming(0_s);
}

void EnableNodeControl::onReceiveNodeControlMessage(NodeControlMessagePointer message) {
    if (message->node != ros::this_node::getName())
        return;
    if (message->node == NodeControlBehavior::resume) {
        setNodeState(NodeState::running);
    }
    else if (message->node == NodeControlBehavior::pause) {
        setNodeState(NodeState::idle);
    }
    else {
        throw std::runtime_error{"Not supported control."};
    }
}

void EnableNodeControl::run() {
    ros::Rate loopRate{getCheckStateFrequency()};
    while (ros::ok()) {
        if (getNodeState() == NodeState::running) {
            whenIsRunning();
            ros::spinOnce();
        }
        else if (getNodeState() == NodeState::idle) {
            whenIsIdle();
        }
        else {
            throw std::runtime_error{"Not supported control."};
        }
        loopRate.sleep();
    }
}
