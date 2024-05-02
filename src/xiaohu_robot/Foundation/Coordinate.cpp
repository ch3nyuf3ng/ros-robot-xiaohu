#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include <sstream>

namespace xiaohu_robot {
inline namespace Foundation {
std::string Coordinate::toString() const {
    std::ostringstream oss;
    oss << "Coordinate{x: " << positionX << ", y: " << positionY << ", z: " << positionZ << "}";
    return oss.str();
}

CoordinateMessage Coordinate::toMessage() const {
    CoordinateMessage message;
    message.position.x = positionX;
    message.position.y = positionY;
    message.position.z = positionZ;
    message.orientation.x = orientationX;
    message.orientation.y = orientationY;
    message.orientation.z = orientationZ;
    message.orientation.w = orientationW;
    return message;
}
}  // namespace Foundation
}  // namespace xiaohu_robot