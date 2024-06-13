#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_NODE_CONTROL_HPP
#define XIAOHU_ROBOT_FOUNDATION_NODE_CONTROL_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <cstddef>
#include <functional>

namespace xiaohu_robot {
inline namespace Foundation {
struct Runnable {
    virtual ~Runnable() = default;
    virtual void run() = 0;
};

struct NodeBasicConfigs final {
    std::string nodeNamespace{CommonConfigs::XiaohuRobotNamespace};
    std::size_t messageBufferSize{CommonConfigs::MessageBufferSize};
    Frequency loopFrequency{CommonConfigs::LoopFrequency()};
};

struct NodeTiming final {
    NodeTiming(Frequency loopFrequency);
    Duration const& getCurrentTiming() const;
    void addTimedTask(Duration const& delay, std::function<void()> task, std::string description);
    void setCurrentTiming(Duration);
    void increment();
    void reset();

private:
    struct TimedTask final {
        Duration executionTime;
        std::function<void()> task;
        std::string description;
    };

    Frequency const loopFrequency;
    Duration currentTiming{0_s};
    std::vector<TimedTask> timedTasks{};
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif