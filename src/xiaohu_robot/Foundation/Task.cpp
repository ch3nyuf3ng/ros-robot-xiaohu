#include "xiaohu_robot/Foundation/Task.hpp"

namespace xiaohu_robot {
inline namespace Foundation {
MappingTask::MappingTask(std::string mapName, std::string savePath):
    mapName{std::move(mapName)},
    savePath{std::move(savePath)} {}
}  // namespace Foundation
}  // namespace xiaohu_robot