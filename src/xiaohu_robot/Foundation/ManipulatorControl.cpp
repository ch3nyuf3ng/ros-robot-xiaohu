#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"

using namespace xiaohu_robot;

/* struct ManipulatorControl */
sensor_msgs::JointState ManipulatorControl::createMessage(size_t control_part_quantity) {
    sensor_msgs::JointState message{};

    message.effort.resize(control_part_quantity);
    message.name.resize(control_part_quantity);
    message.position.resize(control_part_quantity);
    message.velocity.resize(control_part_quantity);

    return message;
}

/* struct ArmControl */
ArmControl::ArmControl(Measurement<UnitLength> const& liftHeight, Measurement<UnitSpeed> const& liftSpeed):
    liftHeight{liftHeight},
    liftSpeed{liftSpeed} {}

sensor_msgs::JointState ArmControl::toMessage() const {
    sensor_msgs::JointState controlMessage{createMessage(1)};

    controlMessage.name[0] = "lift";
    controlMessage.position[0] = liftHeight.getBaseUnitValue();
    controlMessage.velocity[0] = liftSpeed.getBaseUnitValue();

    return controlMessage;
}

std::string ArmControl::toString() const {
    return "ArmControl{liftHeight: " + liftHeight.toString() + ", liftSpeed: " + liftSpeed.toString() + "}";
}

GripperControl::GripperControl(
    Measurement<UnitLength> const& gripper_finger_gap, Measurement<UnitAngularSpeed> const& gripper_move_speed
):
    fingerGap{gripper_finger_gap},
    moveSpeed{gripper_move_speed} {}

sensor_msgs::JointState GripperControl::toMessage() const {
    sensor_msgs::JointState controlMessage{createMessage(1)};

    controlMessage.name[0] = "gripper";
    controlMessage.position[0] = fingerGap.getBaseUnitValue();
    controlMessage.velocity[0] = moveSpeed.getBaseUnitValue();

    return controlMessage;
}

std::string GripperControl::toString() const {
    return "GripperControl{fingerGap: " + fingerGap.toString() + ", moveSpeed: " + moveSpeed.toString() + "}";
}