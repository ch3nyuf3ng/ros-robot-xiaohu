#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <sstream>
#include <utility>

namespace xiaohu_robot {
inline namespace Foundation {
NodeBasicConfig::NodeBasicConfig(std::string nodeNamespace, std::size_t messageBufferSize, Frequency loopFrequency):
    nodeNamespace{std::move(nodeNamespace)},
    messageBufferSize{messageBufferSize},
    loopFrequency{std::move(loopFrequency)} {}

std::string NodeBasicConfig::toString() const {
    std::ostringstream oss;
    oss << "nodeNamespace: " << nodeNamespace << "\n"
        << "messageBufferSize: " << messageBufferSize << "\n"
        << "loopFrequency: " << loopFrequency;
    return oss.str();
}

NodeTiming::NodeTiming(NodeBasicConfig const& nodeBasicConfig):
    nodeBasicConfig{nodeBasicConfig},
    timing{0_s} {}

Duration NodeTiming::getTiming() const {
    return timing;
}

void NodeTiming::setTiming(Duration timing) {
    this->timing = timing;
}

void NodeTiming::incrementTiming() {
    setTiming(getTiming() + nodeBasicConfig.loopFrequency.perCycleTime());
}

void NodeTiming::resetTiming() {
    setTiming(0_s);
}
}  // namespace Foundation
}  // namespace xiaohu_robot