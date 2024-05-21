#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include <sstream>
#include <utility>

namespace xiaohu_robot {
inline namespace Foundation {
MappingTask::MappingTask(MappingTaskMessagePointer message):
    taskId{message->taskId} {}

std::string MappingTask::toString() const {
    std::ostringstream oss;
    oss << "建图任务(" << taskId << ")";
    return oss.str();
}

TaskType MappingTask::getTaskType() const {
    return TaskType::Mapping;
}

InspectionTask::InspectionTask(InspectionTaskMessagePointer message):
    taskId{message->taskId},
    patientName{message->patientName},
    patientPosition{message->patientPosition} {}

std::string InspectionTask::toString() const {
    std::ostringstream oss;
    oss << "巡诊任务(" << taskId << ") {\n";
    oss << "患者姓名：" << patientName << ",\n";
    oss << "患者位置：" << patientPosition.toString() << "\n";
    oss << "}";
    return oss.str();
}

TaskType InspectionTask::getTaskType() const {
    return TaskType::Inspection;
}

MedicineDeliveryTask::MedicineDeliveryTask(MedicineDeliveryTaskMessagePointer message):
    taskId{message->taskId},
    prescription{message->prescription},
    pharmacyName{message->pharmacyName},
    pharmacyPosition{message->pharmacyPosition},
    patientName{message->patientName},
    patientPosition{message->patientPosition} {}

std::string MedicineDeliveryTask::toString() const {
    std::ostringstream oss;
    oss << "送药任务(" << taskId << ") {\n";
    oss << "医嘱：" << prescription << ",\n";
    oss << "药房名称：" << pharmacyName << ",\n";
    oss << "药房位置：" << pharmacyPosition.toString() << "\n";
    oss << "患者姓名：" << patientName << ",\n";
    oss << "患者位置：" << patientPosition.toString() << "\n";
    return oss.str();
}

TaskType MedicineDeliveryTask::getTaskType() const {
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
    if (type == "Mapping") {
        return TaskType::Mapping;
    } else if (type == "Inspection") {
        return TaskType::Inspection;
    } else {
        return TaskType::MedicineDelivery;
    }
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
    return "";
}
}  // namespace Foundation
}  // namespace xiaohu_robot