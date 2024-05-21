#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_GENERICS_HPP
#define XIAOHU_ROBOT_FOUNDATION_GENERICS_HPP

#include <type_traits>

namespace xiaohu_robot {
inline namespace Foundation {
inline namespace Generics {
template<typename Base, typename DerivedUniquePtr>
using isBaseOfUniquePtr
    = std::is_base_of<Base, typename std::remove_reference<decltype(*std::declval<DerivedUniquePtr>())>::type>;
template<typename Base, typename... Derived> struct AreBaseOfUniquePtr;

template<typename Base> struct AreBaseOfUniquePtr<Base>: std::true_type {};

template<typename Base, typename First, typename... Rest>
struct AreBaseOfUniquePtr<Base, First, Rest...>:
    std::integral_constant<bool, isBaseOfUniquePtr<Base, First>::value && AreBaseOfUniquePtr<Base, Rest...>::value> {};
}  // namespace Generics
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif