#pragma once

#ifndef XIAOHU_ROBOT_MANIPULATOR_CONTROL_HPP
#define XIAOHU_ROBOT_MANIPULATOR_CONTROL_HPP

#include "sensor_msgs/JointState.h"
#include "xiaohu_robot/foundation/measurement.hpp"

namespace xiaohu_robot {
inline namespace foundation {
struct ManipulatorControl {
    virtual ~ManipulatorControl() = default;

    virtual sensor_msgs::JointState to_message() const = 0;
    static sensor_msgs::JointState create_message(size_t control_part_quantity);
};

struct ArmControl final: public ManipulatorControl {
    Measurement<UnitLength> const lift_height;
    Measurement<UnitSpeed> const lift_speed;

    ArmControl(Measurement<UnitLength> const& lift_height, Measurement<UnitSpeed> const& lift_speed = 50_cmPs);

    sensor_msgs::JointState to_message() const override;
};

struct GripperControl final: public ManipulatorControl {
    Measurement<UnitLength> const gripper_finger_gap;
    Measurement<UnitAngularSpeed> const gripper_move_speed;

    GripperControl(
        Measurement<UnitLength> const& gripper_finger_gap,
        Measurement<UnitAngularSpeed> const& gripper_move_speed = 5_degPs
    );

    sensor_msgs::JointState to_message() const override;
};
}  // namespace foundation
}  // namespace xiaohu_robot
#endif