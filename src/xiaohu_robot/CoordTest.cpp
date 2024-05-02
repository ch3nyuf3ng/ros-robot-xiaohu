#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include <cstdio>

using namespace xiaohu_robot;

int main() {
    Coordinate coord1{1,1,1};
    Coordinate coord2;
    coord2 = coord1;
    std::printf("%s\n", coord2.toString().c_str());
}