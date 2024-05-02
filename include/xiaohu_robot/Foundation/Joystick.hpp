#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_JOYSTICK_HPP
#define XIAOHU_ROBOT_FOUNDATION_JOYSTICK_HPP

#include <cstddef>

namespace xiaohu_robot {
inline namespace Foundation {
enum JoystickAxis : std::size_t {
    leftStickLeftRightAxis,
    leftStickUpDownAxis,
    leftTrigger,
    rightStickLeftRightAxis,
    rightStickUpDownAxis,
    rightTrigger,
    crossKeyLeftRightAxis,
    crossKeyUpDownAxis
};

enum JoystickButton : std::size_t {
    a,
    b,
    x,
    y,
    leftBuffer,
    rightBuffer,
    back,
    start,
    power,
    leftStickButton,
    rightStickButton
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif