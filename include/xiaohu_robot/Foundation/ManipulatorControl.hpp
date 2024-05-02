#pragma once

#ifndef XIAOHU_ROBOT_MANIPULATOR_CONTROL_HPP
#define XIAOHU_ROBOT_MANIPULATOR_CONTROL_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"


namespace xiaohu_robot {
inline namespace Foundation {
struct ManipulatorControl: public Printable, public MessageConvertible<ManipulatorControlMessage> {
    virtual ~ManipulatorControl() = default;

    static ManipulatorControlMessage createMessage(size_t control_part_quantity);
};

struct ArmControl final: public ManipulatorControl {
    Length const liftHeight;
    LinearSpeed const liftSpeed;

    ArmControl(Length const& liftHeight, LinearSpeed const& liftSpeed = 50_cm_per_s);

    ManipulatorControlMessage toMessage() const override;
    std::string toString() const override;
};

struct GripperControl final: public ManipulatorControl {
    Length const fingerGap;
    AngularSpeed const moveSpeed;

    GripperControl(
        Length const& fingerGap, AngularSpeed const& moveSpeed = 5_deg_per_s
    );

    ManipulatorControlMessage toMessage() const override;
    std::string toString() const override;
};
}  // namespace Foundation
}  // namespace xiaohu_robot
#endif