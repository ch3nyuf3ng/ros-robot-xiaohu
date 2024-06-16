#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_TASK_HPP
#define XIAOHU_ROBOT_FOUNDATION_TASK_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <sstream>
#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
enum struct TaskType {
    Inspection,
    MedicineDelivery
};

enum struct TaskStatus {
    Cancelled,
    Done,
    Failed
};

std::string toString(TaskType);
TaskType toType(std::string);

struct Task: public Printable {
    ~Task() = default;
    virtual TaskType getTaskType() const = 0;
};

struct TaskResult {
    TaskStatus taskStatus;
    std::stringstream logger;
    virtual ~TaskResult() = default;
};

template<typename OriginalTaskType, typename TaskResultMessageType>
struct TaskResultImplementation: public MessageConvertible<TaskResultMessageType>, public TaskResult {
    OriginalTaskType const& task;

    TaskResultImplementation(OriginalTaskType const& task):
        task{task} {}

    virtual ~TaskResultImplementation() = default;
    virtual TaskResultMessageType toMessage() const override {
        TaskResultMessageType result;
        result.taskRequest = task.toMessage();
        result.statusAndDescription = {};
        switch (taskStatus) {
        case TaskStatus::Cancelled:
            result.statusAndDescription.status = StatusAndDescriptionMessage::cancelled;
            break;
        case TaskStatus::Done:
            result.statusAndDescription.status = StatusAndDescriptionMessage::done;
            break;
        case TaskStatus::Failed:
            result.statusAndDescription.status = StatusAndDescriptionMessage::failed;
            break;
        }
        result.statusAndDescription.description = logger.str();
        return result;
    }
};

struct InspectionTask final: public Task, public MessageConvertible<InspectionTaskRequestMessage> {
    InspectionTaskRequestMessage requestMessage;
    std::string taskId;
    std::string patientName;
    Coordinate patientPosition;

    InspectionTask(InspectionTaskRequestMessage::ConstPtr const& message);
    std::string toString() const override;
    TaskType getTaskType() const override;
    InspectionTaskRequestMessage toMessage() const override;

    struct Result final: public TaskResultImplementation<InspectionTask, InspectionTaskResultMessage> {
        bool calledDoctor{false};
        bool measuredTemperature{false};
        Temperature patientTemperature{Temperature(0, UnitTemperature::celcius)};

        using TaskResultImplementation::TaskResultImplementation;
        InspectionTaskResultMessage toMessage() const override;
    };
};

struct MedicineDeliveryTask final: public Task, public MessageConvertible<MedicineDeliveryTaskRequestMessage> {
    MedicineDeliveryTaskRequestMessage requestMessage;
    std::string taskId;
    std::string prescription;
    std::string pharmacyName;
    Coordinate pharmacyPosition;
    std::string patientName;
    Coordinate patientPosition;

    MedicineDeliveryTask(MedicineDeliveryTaskRequestMessage::ConstPtr const& message);
    std::string toString() const override;
    TaskType getTaskType() const override;
    MedicineDeliveryTaskRequestMessage toMessage() const override;

    struct Result final: public TaskResultImplementation<MedicineDeliveryTask, MedicineDeliveryTaskResultMessage> {
        bool fetchedMedicine{false};
        bool deliveredMedicine{false};

        using TaskResultImplementation::TaskResultImplementation;
        MedicineDeliveryTaskResultMessage toMessage() const override;
    };
};

}  // namespace Foundation
}  // namespace xiaohu_robot

#endif