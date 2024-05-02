#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_COORDINATE_HPP
#define XIAOHU_ROBOT_FOUNDATION_COORDINATE_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <type_traits>

namespace xiaohu_robot {
inline namespace Foundation {
struct Coordinate final: public Printable, public MessageConvertible<CoordinateMessage> {
    double positionX, positionY, positionZ;
    double orientationX, orientationY, orientationZ, orientationW;

    Coordinate():
        positionX{},
        positionY{},
        positionZ{},
        orientationX{},
        orientationY{},
        orientationZ{},
        orientationW{} {};

    Coordinate(CoordinateMessagePointer message):
        positionX{message->position.x},
        positionY{message->position.y},
        positionZ(message->position.z),
        orientationX{message->orientation.x},
        orientationY{message->orientation.y},
        orientationZ{message->orientation.z},
        orientationW{message->orientation.w} {}

    template<typename NumberType, typename = std::enable_if_t<std::is_arithmetic<NumberType>::value>>
    Coordinate(NumberType positionX, NumberType positionY, NumberType positionZ):
        positionX{static_cast<double>(positionX)},
        positionY{static_cast<double>(positionY)},
        positionZ{static_cast<double>(positionZ)},
        orientationX{},
        orientationY{},
        orientationZ{},
        orientationW{} {}

    template<typename NumberType, typename = std::enable_if_t<std::is_arithmetic<NumberType>::value>>
    Coordinate(
        NumberType positionX,
        NumberType positionY,
        NumberType positionZ,
        NumberType orientationX,
        NumberType orientationY,
        NumberType orientationZ,
        NumberType orientationW
    ):
        positionX{static_cast<double>(positionX)},
        positionY{static_cast<double>(positionY)},
        positionZ{static_cast<double>(positionZ)},
        orientationX{static_cast<double>(orientationX)},
        orientationY{static_cast<double>(orientationY)},
        orientationZ{static_cast<double>(orientationZ)},
        orientationW{static_cast<double>(orientationW)} {}

    std::string toString() const override;
    CoordinateMessage toMessage() const override;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif