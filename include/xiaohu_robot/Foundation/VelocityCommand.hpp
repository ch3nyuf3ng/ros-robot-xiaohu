#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_VELOCITY_COMMAND_HPP
#define XIAOHU_ROBOT_FOUNDATION_VELOCITY_COMMAND_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"

namespace xiaohu_robot {
inline namespace Foundation {
struct VelocityCommand final: public Printable, public MessageConvertible<VelocityCommandMessage> {
    LinearSpeed forwardBackwardAxis;
    LinearSpeed leftRightAxis;
    AngularSpeed rotationAxis;

    VelocityCommand(LinearSpeed x, LinearSpeed y, AngularSpeed z);
    VelocityCommand(JoystickMessage::ConstPtr const& message, LinearSpeed maxLinearSpeed, AngularSpeed maxAngularSpeed);
    VelocityCommandMessage toMessage() const override;
    std::string toString() const override;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif