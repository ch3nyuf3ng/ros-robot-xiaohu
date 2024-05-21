#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_EXCEPTIONS_HPP
#define XIAOHU_ROBOT_FOUNDATION_EXCEPTIONS_HPP

#include <stdexcept>
#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
struct RuntimeErrorWithErrorCode: public std::runtime_error {
    RuntimeErrorWithErrorCode(std::string reason, int errorCode):
        std::runtime_error(reason + "，错误码：" + std::to_string(errorCode)) {}
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif