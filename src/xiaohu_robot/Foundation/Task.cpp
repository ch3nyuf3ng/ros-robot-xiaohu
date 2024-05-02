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

TaskType MedicineDeliveryTasks::getTaskType() const {
    return TaskType::MedicineDelivery;
}

LegacyGeneralTask::LegacyGeneralTask(
    TaskType type,
    std::string mapName,
    std::string savePath,
    std::string pharmacy,
    std::string patient,
    std::string prescription,
    Coordinate medicinePosition
):
    type{type},
    mapName{std::move(mapName)},
    savePath{std::move(savePath)},
    pharmacy{std::move(pharmacy)},
    patient{std::move(patient)},
    prescription{std::move(prescription)} {}

LegacyGeneralTask::LegacyGeneralTask(GeneralTaskMessagePointer message):
    type{toType(message->type)},
    mapName{std::move(message->mapName)},
    savePath{std::move(message->savePath)},
    pharmacy{std::move(message->pharmacy)},
    patient{std::move(message->patient)},
    prescription{std::move(message->prescription)} {}

std::string LegacyGeneralTask::toString() const {
    std::ostringstream oss;
    oss << "LegacyGeneralTask{\n"
        << "type: " << xiaohu_robot::toString(type) << ",\n"
        << "mapName: " << (mapName.empty() ? "NULL" : mapName) << ",\n"
        << "savePath: " << (savePath.empty() ? "NULL" : savePath) << ",\n"
        << "pharmacy: " << (pharmacy.empty() ? "NULL" : pharmacy) << ",\n"
        << "patient: " << (patient.empty() ? "NULL" : patient) << ",\n"
        << "prescription: " << (prescription.empty() ? "NULL" : prescription) << "\n"
        << "}";
    return oss.str();
}

TaskType LegacyGeneralTask::getTaskType() const {
    return type;
}

TaskType toType(std::string type) {
    if (type == "Mapping")
        return TaskType::Mapping;
    else if (type == "Inspection")
        return TaskType::Inspection;
    else
        return TaskType::MedicineDelivery;
}

std::string toString(TaskType type) {
    switch (type) {
    case TaskType::Mapping:
        return "MappingTask";
    case TaskType::Inspection:
        return "InspectionTasks";
    case TaskType::MedicineDelivery:
        return "MedicineDeliveryTasks";
    }
}
}  // namespace Foundation
}  // namespace xiaohu_robot