#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <functional>
#include <utility>

namespace xiaohu_robot {
inline namespace Foundation {
NodeTiming::NodeTiming(Frequency loopFrequency):
    loopFrequency{std::move(loopFrequency)} {}

Duration const& NodeTiming::getCurrentTiming() const {
    return currentTiming;
}

void NodeTiming::addTimedTask(Duration const& delay, std::function<void()> task, std::string description) {
    Duration executionTime{delay + getCurrentTiming()};
    timedTasks.push_back({std::move(executionTime), std::move(task), description});
    std::cout << '[' << getCurrentTiming() << "] 将在" << delay << "后执行任务：" << description << std::endl;
}

void NodeTiming::setCurrentTiming(Duration timing) {
    this->currentTiming = std::move(timing);
}

void NodeTiming::increment() {
    setCurrentTiming(getCurrentTiming() + loopFrequency.perCycleTime());
    auto removeIter = std::remove_if(timedTasks.begin(), timedTasks.end(), [&](TimedTask const& timedTask) {
        if (getCurrentTiming() >= timedTask.executionTime) {
            timedTask.task();
            std::cout << '[' << getCurrentTiming() << "] 已执行任务：" << timedTask.description << std::endl;
            return true;
        } else {
            return false;
        }
    });
    timedTasks.erase(removeIter, timedTasks.end());
}

void NodeTiming::reset() {
    timedTasks.clear();
    setCurrentTiming(0_s);
}
}  // namespace Foundation
}  // namespace xiaohu_robot