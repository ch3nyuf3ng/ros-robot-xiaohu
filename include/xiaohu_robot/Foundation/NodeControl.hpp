#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_NODE_CONTROL_HPP
#define XIAOHU_ROBOT_FOUNDATION_NODE_CONTROL_HPP

#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
enum struct NodeState {
    idle,
    running
};

struct NodeControlBehavior {
    std::string const static pause;
    std::string const static resume;
};

struct NodeRunnable {
    virtual ~NodeRunnable() = default;
    virtual void run() = 0;

protected:
    virtual Frequency getCheckStateFrequency() const = 0;
    virtual Duration getTiming() const = 0;
    virtual void setTiming(Duration) = 0;
    virtual void incrementTiming() = 0;
    virtual void resetTiming() = 0;
};

struct NodeControllable: public NodeRunnable {
    virtual ~NodeControllable() = default;

protected:
    virtual NodeState getNodeState() const = 0;
    virtual void setNodeState(NodeState) = 0;
    virtual void onReceiveNodeControlMessage(NodeControlMessagePointer) = 0;
    virtual void whenIsRunning() = 0;
    virtual void whenIsIdle() = 0;
};

struct EnableNodeControl: public NodeControllable {
    virtual ~EnableNodeControl() = default;
    void run() final;

protected:
    void incrementTiming() final;
    void resetTiming() final;
    void onReceiveNodeControlMessage(NodeControlMessagePointer) final;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif