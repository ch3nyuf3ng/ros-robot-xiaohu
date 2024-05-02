#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_TASK_HPP
#define XIAOHU_ROBOT_FOUNDATION_TASK_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include <string>
#include <vector>

namespace xiaohu_robot {
inline namespace Foundation {
struct SpecificTask: public Printable {
    enum struct Type {
        MappingTask,
        InspectionTasks,
        MedicineDeliveryTasks
    };
    ~SpecificTask() = default;
    virtual Type getTaskType() const = 0;
};

struct MappingTask final: public SpecificTask {
    std::string mapName;
    std::string savePath;

    MappingTask(std::string mapName, std::string savePath);
    std::string toString() const override;
    Type getTaskType() const override;
};

struct InspectionTasks final: public SpecificTask {
    std::vector<Coordinate> patients;

    InspectionTasks(std::vector<Coordinate> patients);
    std::string toString() const override;
    Type getTaskType() const override;
};

struct MedicineDeliverySingleTask final {
    Coordinate pharmacy;
    Coordinate patient;

    MedicineDeliverySingleTask(Coordinate pharmacy, Coordinate patient);
};

struct MedicineDeliveryTasks final: public SpecificTask {
    std::vector<MedicineDeliverySingleTask> tasks;

    MedicineDeliveryTasks(std::vector<MedicineDeliverySingleTask> tasks);
    std::string toString() const override;
    Type getTaskType() const override;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif