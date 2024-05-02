#include "xiaohu_robot/Foundation/VelocityCommand.hpp"
#include "xiaohu_robot/Foundation/Joystick.hpp"
#include <sstream>

namespace xiaohu_robot {
inline namespace Foundation {
VelocityCommand::VelocityCommand(LinearSpeed forwardBackwardAxis, LinearSpeed leftRightAxis, AngularSpeed rotationAxis):
    forwardBackwardAxis{std::move(forwardBackwardAxis)},
    leftRightAxis{std::move(leftRightAxis)},
    rotationAxis{std::move(rotationAxis)} {}

VelocityCommand::VelocityCommand(
    JoystickMessagePointer message, LinearSpeed maxLinearSpeed, AngularSpeed maxAngularSpeed
):
    forwardBackwardAxis{maxLinearSpeed * message->axes[leftStickUpDownAxis]},
    leftRightAxis{maxLinearSpeed * message->axes[leftStickLeftRightAxis]},
    rotationAxis{maxAngularSpeed * message->axes[rightStickLeftRightAxis]} {}

VelocityCommandMessage VelocityCommand::toMessage() const {
    VelocityCommandMessage message;
    message.linear.x = forwardBackwardAxis.getBaseUnitValue();
    message.linear.y = leftRightAxis.getBaseUnitValue();
    message.angular.z = rotationAxis.getBaseUnitValue();
    return message;
}

std::string VelocityCommand::toString() const {
    std::ostringstream oss;
    oss << "LinearSpeed x: " << forwardBackwardAxis << ", LinearSpeed y: " << leftRightAxis
        << ", AngularSpeed z: " << rotationAxis;
    return oss.str();
}
}  // namespace Foundation
}  // namespace xiaohu_robot