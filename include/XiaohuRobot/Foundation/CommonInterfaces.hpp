#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_COMMON_INTERFACES_HPP
#define XIAOHU_ROBOT_FOUNDATION_COMMON_INTERFACES_HPP

#include <string>

namespace XiaohuRobot {
inline namespace Foundation {
template<typename T> class Equatable {
public:
    virtual ~Equatable() = default;
    virtual bool equals(T const& object) const = 0;
};

class Printable {
public:
    virtual ~Printable() = default;
    virtual std::string toString() const = 0;
};
}  // namespace foundation
}  // namespace xiaohu_robot
#endif