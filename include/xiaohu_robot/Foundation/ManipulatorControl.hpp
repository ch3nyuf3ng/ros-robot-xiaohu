#pragma once

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#ifndef XIAOHU_ROBOT_MANIPULATOR_CONTROL_HPP
#define XIAOHU_ROBOT_MANIPULATOR_CONTROL_HPP

#include "sensor_msgs/JointState.h"
#include "xiaohu_robot/Foundation/Measurement.hpp"

namespace xiaohu_robot {
inline namespace Foundation {
struct ManipulatorControl: public Printable {
    virtual ~ManipulatorControl() = default;

    virtual sensor_msgs::JointState toMessage() const = 0;
    static sensor_msgs::JointState createMessage(size_t control_part_quantity);
};

struct ArmControl final: public ManipulatorControl {
    Measurement<UnitLength> const liftHeight;
    Measurement<UnitSpeed> const liftSpeed;

    ArmControl(Measurement<UnitLength> const& liftHeight, Measurement<UnitSpeed> const& liftSpeed = 50_cm_per_s);

    sensor_msgs::JointState toMessage() const override;
    std::string toString() const override;
};

struct GripperControl final: public ManipulatorControl {
    Measurement<UnitLength> const fingerGap;
    Measurement<UnitAngularSpeed> const moveSpeed;

    GripperControl(
        Measurement<UnitLength> const& fingerGap,
        Measurement<UnitAngularSpeed> const& moveSpeed = 5_deg_per_s
    );

    sensor_msgs::JointState toMessage() const override;
    std::string toString() const override;
};
}  // namespace foundation
}  // namespace xiaohu_robot
#endif