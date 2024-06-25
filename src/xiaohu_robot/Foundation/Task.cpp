#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Exceptions.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <sstream>

namespace xiaohu_robot {
inline namespace Foundation {
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

MedicineDeliveryTaskRequestMessage MedicineDeliveryTask::toMessage() const {
    return requestMessage;
}

MedicineDeliveryTaskResultMessage MedicineDeliveryTask::Result::toMessage() const {
    MedicineDeliveryTaskResultMessage result{TaskResultImplementation::toMessage()};
    result.deliveredMedicine = deliveredMedicine;
    result.fetchedMedicine = fetchedMedicine;
    return result;
}

TaskType toType(std::string type) {
    if (type == "Inspection") {
        return TaskType::Inspection;
    } else if (type == "MedicineDelivery") {
        return TaskType::MedicineDelivery;
    } else {
        printMessageThenThrowRuntimeError("未支持的任务类型");
    }
}

std::string toString(TaskType type) {
    switch (type) {
    case TaskType::Inspection:
        return "巡检任务";
    case TaskType::MedicineDelivery:
        return "送药任务";
    }
    return "";
}
}  // namespace Foundation
}  // namespace xiaohu_robot