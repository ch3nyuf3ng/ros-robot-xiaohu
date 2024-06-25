#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <sstream>

namespace xiaohu_robot {
inline namespace Foundation {
std::string Coordinate::toString() const {
    std::ostringstream oss;
    oss << "Position{x: " << positionX << ", y: " << positionY << ", z: " << positionZ << "}\n"
        << "Oreientation{x: " << orientationX << ", y" << orientationY << ", z: " << orientationZ
        << ", w: " << orientationW << "}";
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

NavigationGoal Coordinate::toNavigationGoal() const {
    NavigationGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = positionX;
    goal.target_pose.pose.position.y = positionY;
    goal.target_pose.pose.position.z = positionZ;
    goal.target_pose.pose.orientation.x = orientationX;
    goal.target_pose.pose.orientation.y = orientationY;
    goal.target_pose.pose.orientation.z = orientationZ;
    goal.target_pose.pose.orientation.w = orientationW;
    return goal;
}
}  // namespace Foundation
}  // namespace xiaohu_robot