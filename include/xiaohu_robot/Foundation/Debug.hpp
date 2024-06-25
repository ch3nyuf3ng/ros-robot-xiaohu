#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_DEBUG_HPP
#define XIAOHU_ROBOT_FOUNDATION_DEBUG_HPP

#include <cstdint>
#include <iomanip>
#include <iostream>

namespace xiaohu_robot {
inline namespace Foundation {
template<typename T> void printElement(T&& element, std::true_type) {
    std::cout << "0x" << std::forward<T>(element) << " ";
}

template<typename T> void printElement(T&& element, std::false_type) {
    std::cout << std::forward<T>(element) << " ";
}

inline void printElement(std::uint8_t element) {
    std::cout << "0x" << std::hex << std::setw(2) << std::setfill('0') << std::uppercase
              << static_cast<unsigned int>(element) << " ";
}

template<typename T> void printElement(T&& element) {
    printElement(std::forward<T>(element), std::is_integral<std::decay_t<T>>{});
}

template<typename Container> void printContainer(Container&& container) {
    for (auto&& element : std::forward<Container>(container)) {
        printElement(std::forward<decltype(element)>(element));
    }
    std::cout << std::endl;
}

template<typename Container> void printContainer(std::string const& hint, Container&& container) {
    std::cerr << hint << std::endl;
    printContainer(container);
}
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif