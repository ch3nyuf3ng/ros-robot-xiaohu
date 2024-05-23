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

    Coordinate(CoordinateMessage::ConstPtr const& message):
        Coordinate(*message) {}

    Coordinate(CoordinateMessage message):
        positionX{std::move(message.position.x)},
        positionY{std::move(message.position.y)},
        positionZ(std::move(message.position.z)),
        orientationX{std::move(message.orientation.x)},
        orientationY{std::move(message.orientation.y)},
        orientationZ{std::move(message.orientation.z)},
        orientationW{std::move(message.orientation.w)} {}

    template<typename NumberType, typename = std::enable_if_t<std::is_arithmetic<NumberType>::value>>
    Coordinate(NumberType x, NumberType y, NumberType z):
        positionX{static_cast<double>(x)},
        positionY{static_cast<double>(y)},
        positionZ{static_cast<double>(z)},
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

    Coordinate& operator=(Coordinate other) {
        this->positionX = other.positionX;
        this->positionY = other.positionY;
        this->positionZ = other.positionZ;
        this->orientationX = other.orientationX;
        this->orientationY = other.orientationY;
        this->orientationZ = other.orientationZ;
        this->orientationW = other.orientationW;
        return *this;
    }

    std::string toString() const override;
    CoordinateMessage toMessage() const override;
    NavigationGoal toNavigationGoal() const;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif