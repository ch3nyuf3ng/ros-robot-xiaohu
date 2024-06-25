#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_COMMON_INTERFACES_HPP
#define XIAOHU_ROBOT_FOUNDATION_COMMON_INTERFACES_HPP

#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
template<typename T> struct Equatable {
    virtual ~Equatable() = default;
    virtual bool equals(T const& object) const = 0;
};

struct Printable {
    virtual ~Printable() = default;
    virtual std::string toString() const = 0;
};

template<typename MessageType> struct MessageConvertible {
    virtual ~MessageConvertible() = default;
    virtual MessageType toMessage() const = 0;
};
}  // namespace Foundation
}  // namespace xiaohu_robot
#endif