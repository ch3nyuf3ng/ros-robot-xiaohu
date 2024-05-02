#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_TASK_HPP
#define XIAOHU_ROBOT_FOUNDATION_TASK_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <deque>
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
    enum struct Type {
        Mapping,
        Inspection,
        MedicineDelivery
    };
    ~SpecificTask() = default;
    virtual TaskType getTaskType() const = 0;
};

struct MappingTask final: public SpecificTask {
    std::string mapName;
    std::string savePath;

    MappingTask(std::string mapName, std::string savePath);
    MappingTask(MappingTaskMessagePointer message);
    std::string toString() const override;
    TaskType getTaskType() const override;
};

struct InspectionTasks final: public SpecificTask {
    std::deque<Coordinate> patients;

    InspectionTasks(std::vector<Coordinate> patients);
    std::string toString() const override;
    TaskType getTaskType() const override;
};

struct MedicineDeliverySingleTask final: public Printable {
    Coordinate pharmacy;
    Coordinate patient;

    MedicineDeliverySingleTask(Coordinate pharmacy, Coordinate patient);
    MedicineDeliverySingleTask(MedicineDeliverySingleTaskMessage);
    std::string toString() const override;
};

struct MedicineDeliveryTasks final: public SpecificTask {
    std::deque<MedicineDeliverySingleTask> tasks;

    MedicineDeliveryTasks(MedicineDeliveryTasksMessagePointer);
    std::string toString() const override;
    TaskType getTaskType() const override;
};

struct LegacyGeneralTask final: public Printable {
    enum struct Type {
        Mapping,
        Inspection,
        MedicineDelivery
    };

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