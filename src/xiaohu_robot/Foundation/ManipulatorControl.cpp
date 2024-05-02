#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"

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
ArmControl::ArmControl(Length const& liftHeight, LinearSpeed const& liftSpeed):
    liftHeight{liftHeight},
    liftSpeed{liftSpeed} {}

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

GripperControl::GripperControl(Length const& gripper_finger_gap, AngularSpeed const& gripper_move_speed):
    fingerGap{gripper_finger_gap},
    moveSpeed{gripper_move_speed} {}

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
}  // namespace Foundation
}  // namespace xiaohu_robot