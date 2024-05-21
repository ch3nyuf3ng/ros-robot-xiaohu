#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <cstddef>
#include <memory>
#include <sstream>
#include <utility>

namespace xiaohu_robot {
inline namespace Foundation {
/* struct ManipulatorControl */
ManipulatorControlMessage ManipulatorControl::createMessage(std::size_t control_part_quantity) {
    ManipulatorControlMessage message{};

    message.effort.resize(control_part_quantity);
    message.name.resize(control_part_quantity);
    message.position.resize(control_part_quantity);
    message.velocity.resize(control_part_quantity);

    return message;
}

/* struct ArmControl */
ArmControl::ArmControl(Length liftHeight, LinearSpeed liftSpeed):
    liftHeight{std::move(liftHeight)},
    liftSpeed{std::move(liftSpeed)} {}

ManipulatorControlMessage ArmControl::toMessage() const {
    ManipulatorControlMessage controlMessage{createMessage(1)};

    controlMessage.name[0] = "lift";
    controlMessage.position[0] = liftHeight.getBaseUnitValue();
    controlMessage.velocity[0] = liftSpeed.getBaseUnitValue();

    return controlMessage;
}

std::string ArmControl::toString() const {
    return "ArmControl{liftHeight: " + liftHeight.toString() + ", liftSpeed: " + liftSpeed.toString() + "}";
}

GripperControl::GripperControl(Length fingerGap, AngularSpeed moveSpeed):
    fingerGap{std::move(fingerGap)},
    moveSpeed{std::move(moveSpeed)} {}

ManipulatorControlMessage GripperControl::toMessage() const {
    ManipulatorControlMessage controlMessage{createMessage(1)};

    controlMessage.name[0] = "gripper";
    controlMessage.position[0] = fingerGap.getBaseUnitValue();
    controlMessage.velocity[0] = moveSpeed.getBaseUnitValue();

    return controlMessage;
}

std::string GripperControl::toString() const {
    return "GripperControl{fingerGap: " + fingerGap.toString() + ", moveSpeed: " + moveSpeed.toString() + "}";
}

std::string MultiPartControl::toString() const {
    std::ostringstream oss;
    oss << "MultiPartControl{" << '\n';
    for (auto& control : controls) {
        oss << control->toString() << '\n';
    }
    oss << "}";
    return oss.str();
}

ManipulatorControlMessage MultiPartControl::toMessage() const {
    ManipulatorControlMessage controlMessage{createMessage(controls.size())};
    std::size_t index{0};
    for (auto& control : controls) {
        ManipulatorControlMessage partControlMessage{control->toMessage()};
        controlMessage.name[index] = partControlMessage.name[0];
        controlMessage.position[index] = partControlMessage.position[0];
        controlMessage.velocity[index] = partControlMessage.velocity[0];
        controlMessage.effort[index] = partControlMessage.effort[0];
        index++;
    }
    return controlMessage;
}
}  // namespace Foundation
}  // namespace xiaohu_robot