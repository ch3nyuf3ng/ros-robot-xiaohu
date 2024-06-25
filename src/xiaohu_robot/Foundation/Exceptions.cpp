#include "xiaohu_robot/Foundation/Exceptions.hpp"
#include <iostream>
#include <stdexcept>

namespace xiaohu_robot {
inline namespace Foundation {
[[noreturn]] void printMessageThenThrowRuntimeError(std::string const& message) {
    std::cerr << message << std::endl;
    throw std::runtime_error{message};
}
}  // namespace Foundation
}  // namespace xiaohu_robot