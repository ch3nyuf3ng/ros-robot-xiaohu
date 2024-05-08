#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_NODE_CONTROL_HPP
#define XIAOHU_ROBOT_FOUNDATION_NODE_CONTROL_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <cstddef>

namespace xiaohu_robot {
inline namespace Foundation {
struct Runnable {
    virtual ~Runnable() = default;
    virtual void run() = 0;
};

struct NodeBasicConfig final: public Printable {
    std::string nodeNamespace;
    std::size_t messageBufferSize;
    Frequency loopFrequency;

    NodeBasicConfig(
        std::string nodeNamespace = CommonConfigs::nodeNamespace,
        std::size_t messageBufferSize = CommonConfigs::messageBufferSize,
        Frequency loopFrequency = CommonConfigs::loopFrequency
    );
    std::string toString() const override;
};

struct NodeTiming final {
    NodeTiming(NodeBasicConfig const& nodeBasicConfig);
    Duration getTiming() const;
    void setTiming(Duration);
    void incrementTiming();
    void resetTiming();

private:
    Duration timing;
    NodeBasicConfig const& nodeBasicConfig;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif