#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_TASK_HPP
#define XIAOHU_ROBOT_FOUNDATION_TASK_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
enum struct TaskType {
    Mapping,
    Inspection,
    MedicineDelivery
};

std::string toString(TaskType);
TaskType toType(std::string);

struct SpecificTask: public Printable {
    ~SpecificTask() = default;
    virtual TaskType getTaskType() const = 0;
};

struct MappingTask final: public SpecificTask {
    std::string taskId;

    MappingTask(MappingTaskMessagePointer message);
    std::string toString() const override;
    TaskType getTaskType() const override;
};

struct InspectionTask final: public SpecificTask {
    std::string taskId;
    std::string patientName;
    Coordinate patientPosition;

    InspectionTask(InspectionTaskMessagePointer message);
    std::string toString() const override;
    TaskType getTaskType() const override;
};

struct MedicineDeliveryTask final: public SpecificTask {
    std::string taskId;
    std::string prescription;
    std::string pharmacyName;
    Coordinate pharmacyPosition;
    std::string patientName;
    Coordinate patientPosition;

    MedicineDeliveryTask(MedicineDeliveryTaskMessagePointer message);
    std::string toString() const override;
    TaskType getTaskType() const override;
};

struct LegacyGeneralTask final: public Printable {
    TaskType type;
    std::string mapName;
    std::string savePath;
    std::string pharmacy;
    std::string patient;
    std::string prescription;
    Coordinate medicinePosition;

    LegacyGeneralTask(
        TaskType type,
        std::string mapName,
        std::string savePath,
        std::string pharmacy,
        std::string patient,
        std::string prescription,
        Coordinate medicinePosition
    );

    LegacyGeneralTask(GeneralTaskMessagePointer);

    std::string toString() const override;
    TaskType getTaskType() const;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif