#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <sstream>
#include <stdexcept>

namespace xiaohu_robot {
inline namespace Foundation {
MappingTask::MappingTask(MappingTaskRequestMessage::ConstPtr const& message):
    requestMessage{*message},
    taskId{message->taskId} {}

std::string MappingTask::toString() const {
    std::ostringstream content;
    content << "建图任务 ID: " << taskId << '\n';
    return content.str();
}

TaskType MappingTask::getTaskType() const {
    return TaskType::Mapping;
}

TaskVersion MappingTask::getTaskVersion() const {
    return TaskVersion::New;
}

MappingTaskRequestMessage MappingTask::toMessage() const {
    return requestMessage;
}

InspectionTask::InspectionTask(InspectionTaskRequestMessage::ConstPtr const& message):
    requestMessage{*message},
    taskId{message->taskId},
    patientName{message->patientName},
    patientPosition{message->patientPosition} {}

std::string InspectionTask::toString() const {
    std::ostringstream content;
    content << "巡诊任务 ID: " << taskId << '\n';
    content << "患者姓名：" << patientName << ",\n";
    content << "患者位置：" << patientPosition.toString();
    return content.str();
}

TaskType InspectionTask::getTaskType() const {
    return TaskType::Inspection;
}

TaskVersion InspectionTask::getTaskVersion() const {
    return TaskVersion::New;
}

InspectionTaskRequestMessage InspectionTask::toMessage() const {
    return requestMessage;
}

InspectionTaskResultMessage InspectionTask::Result::toMessage() const {
    InspectionTaskResultMessage result{TaskResultImplementation::toMessage()};
    result.calledDoctor = calledDoctor;
    result.measuredTemperature = measuredTemperature;
    result.patientTemperature = patientTemperature;
    return result;
}

MedicineDeliveryTask::MedicineDeliveryTask(MedicineDeliveryTaskRequestMessage::ConstPtr const& message):
    requestMessage{*message},
    taskId{message->taskId},
    prescription{message->prescription},
    pharmacyName{message->pharmacyName},
    pharmacyPosition{message->pharmacyPosition},
    patientName{message->patientName},
    patientPosition{message->patientPosition} {}

std::string MedicineDeliveryTask::toString() const {
    std::ostringstream content;
    content << "送药任务 ID: " << taskId << '\n';
    content << "医嘱: " << prescription << '\n';
    content << "药房名称: " << pharmacyName << '\n';
    content << "药房位置: " << pharmacyPosition.toString() << '\n';
    content << "患者姓名: " << patientName << '\n';
    content << "患者位置: " << patientPosition.toString();
    return content.str();
}

TaskType MedicineDeliveryTask::getTaskType() const {
    return TaskType::MedicineDelivery;
}

TaskVersion MedicineDeliveryTask::getTaskVersion() const {
    return TaskVersion::New;
}

MedicineDeliveryTaskRequestMessage MedicineDeliveryTask::toMessage() const {
    return requestMessage;
}

MedicineDeliveryTaskResultMessage MedicineDeliveryTask::Result::toMessage() const {
    MedicineDeliveryTaskResultMessage result{TaskResultImplementation::toMessage()};
    result.deliveredMedicine = deliveredMedicine;
    result.fetchedMedicine = fetchedMedicine;
    return result;
}

LegacyGeneralTask::LegacyGeneralTask(LegacyGeneralTaskRequestMessage::ConstPtr const& message):
    requestMessage{*message},
    taskId{message->taskId},
    type{toType(message->type)},
    pharmacy{message->pharmacy},
    patient{message->patient},
    prescription{message->prescription} {}

std::string LegacyGeneralTask::toString() const {
    std::ostringstream content;
    content << "老式万能任务 ID:" << taskId << '\n';
    content << "任务类型: " << xiaohu_robot::toString(type) << '\n';
    content << "药房名称: " << (pharmacy.empty() ? "NULL" : pharmacy) << '\n';
    content << "患者姓名: " << (patient.empty() ? "NULL" : patient) << '\n';
    content << "医嘱: " << (prescription.empty() ? "NULL" : prescription);
    return content.str();
}

TaskType LegacyGeneralTask::getTaskType() const {
    return type;
}

TaskVersion LegacyGeneralTask::getTaskVersion() const {
    return TaskVersion::Legacy;
}

LegacyGeneralTaskRequestMessage LegacyGeneralTask::toMessage() const {
    return requestMessage;
}

TaskType toType(std::string type) {
    if (type == "Mapping") {
        return TaskType::Mapping;
    } else if (type == "Inspection") {
        return TaskType::Inspection;
    } else if (type == "MedicineDelivery") {
        return TaskType::MedicineDelivery;
    } else {
        throw std::runtime_error("未支持的任务类型");
    }
}

std::string toString(TaskType type) {
    switch (type) {
    case TaskType::Mapping:
        return "建图任务";
    case TaskType::Inspection:
        return "巡检任务";
    case TaskType::MedicineDelivery:
        return "送药任务";
    }
    return "";
}
}  // namespace Foundation
}  // namespace xiaohu_robot