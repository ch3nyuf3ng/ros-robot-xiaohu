#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include <algorithm>
#include <sstream>
#include <utility>

namespace xiaohu_robot {
inline namespace Foundation {
// MappingTask::MappingTask(std::string mapName, std::string savePath):
//     mapName{std::move(mapName)},
//     savePath{std::move(savePath)} {}

// MappingTask::MappingTask(MappingTaskMessagePointer message):
//     mapName{std::move(message->mapName)},
//     savePath{std::move(message->savePath)} {}

MedicineDeliverySingleTask::MedicineDeliverySingleTask(Coordinate pharmacy, Coordinate patient):
    pharmacy{std::move(pharmacy)},
    patient{std::move(patient)} {}

MedicineDeliverySingleTask::MedicineDeliverySingleTask(MedicineDeliverySingleTaskMessage message):
    pharmacy{std::move(Coordinate{message.pharmacy})},
    patient{std::move(Coordinate{message.patient})} {}

std::string MedicineDeliverySingleTask::toString() const {
    std::ostringstream oss;
    oss << "Pharmacy:\n" << pharmacy.toString() << "Patient:\n" << patient.toString();
    return oss.str();
}

MedicineDeliveryTasks::MedicineDeliveryTasks(MedicineDeliveryTasksMessagePointer message) {
    std::deque<MedicineDeliverySingleTask> tasks;
    std::transform(
        message->tasks.begin(),
        message->tasks.end(),
        tasks.begin(),
        [](MedicineDeliverySingleTaskMessage m) { return MedicineDeliverySingleTask{m}; }
    );
    this->tasks = std::move(tasks);
}

std::string MedicineDeliveryTasks::toString() const {
    std::ostringstream oss;
    oss << "MedicineDeliveryTasks{\n";
    for (MedicineDeliverySingleTask const& task : tasks) {
        oss << task.toString() << "\n";
    }
    oss << "}";
    return oss.str();
}

SpecificTask::Type MedicineDeliveryTasks::getTaskType() const {
    return Type::MedicineDeliveryTasks;
}
}  // namespace Foundation
}  // namespace xiaohu_robot