#include "xiaohu_robot/foundation/manipulator_control.hpp"

using namespace xiaohu_robot;

/* struct ManipulatorControl */
sensor_msgs::JointState ManipulatorControl::create_message(size_t control_part_quantity) {
    sensor_msgs::JointState message{};

    message.effort.resize(control_part_quantity);
    message.name.resize(control_part_quantity);
    message.position.resize(control_part_quantity);
    message.velocity.resize(control_part_quantity);

    return message;
}

/* struct ArmControl */
ArmControl::ArmControl(Measurement<UnitLength> const& lift_height, Measurement<UnitSpeed> const& lift_speed):
    lift_height{lift_height}, lift_speed{lift_speed} {}

sensor_msgs::JointState ArmControl::to_message() const {
    sensor_msgs::JointState arm_control_message{create_message(1)};

    arm_control_message.name[0] = "lift";
    arm_control_message.position[0] = lift_height.get_base_unit_value();
    arm_control_message.velocity[0] = lift_speed.get_base_unit_value();

    return arm_control_message;
}

GripperControl::GripperControl(
    Measurement<UnitLength> const& gripper_finger_gap, Measurement<UnitAngularSpeed> const& gripper_move_speed
): gripper_finger_gap{gripper_finger_gap}, gripper_move_speed{gripper_move_speed} {}

sensor_msgs::JointState GripperControl::to_message() const {
    sensor_msgs::JointState arm_control_message{create_message(1)};

    arm_control_message.name[0] = "gripper";
    arm_control_message.position[0] = gripper_finger_gap.get_base_unit_value();
    arm_control_message.velocity[0] = gripper_move_speed.get_base_unit_value();

    return arm_control_message;
}