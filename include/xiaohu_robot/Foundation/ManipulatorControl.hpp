#pragma once

#ifndef XIAOHU_ROBOT_MANIPULATOR_CONTROL_HPP
#define XIAOHU_ROBOT_MANIPULATOR_CONTROL_HPP

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#include "xiaohu_robot/Foundation/Generics.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <memory>

namespace xiaohu_robot {
inline namespace Foundation {
struct ManipulatorControl: public Printable, public MessageConvertible<ManipulatorControlMessage> {
    virtual ~ManipulatorControl() = default;

    static ManipulatorControlMessage createMessage(size_t control_part_quantity);
};

struct ManipulatorPartControl: public ManipulatorControl {
    virtual ~ManipulatorPartControl() = default;
};

struct ArmControl final: public ManipulatorPartControl {
    Length const liftHeight;
    LinearSpeed const liftSpeed;

    ArmControl(Length liftHeight, LinearSpeed liftSpeed = 50_cm_per_s);

    ManipulatorControlMessage toMessage() const override;
    std::string toString() const override;
};

struct GripperControl final: public ManipulatorPartControl {
    Length const fingerGap;
    AngularSpeed const moveSpeed;

    GripperControl(Length fingerGap, AngularSpeed moveSpeed = 5_deg_per_s);

    ManipulatorControlMessage toMessage() const override;
    std::string toString() const override;
};

struct MultiPartControl final: public ManipulatorControl {
    std::vector<std::unique_ptr<ManipulatorPartControl>> controls;

    template<typename... Args> MultiPartControl(Args&&... args) {
        static_assert(
            AreBaseOfUniquePtr<ManipulatorPartControl, Args...>::value,
            "All arguments must be derived from ManipulatorPartControl"
        );
        controls.reserve(sizeof...(args));
        addControls(std::forward<Args>(args)...);
    }

    ManipulatorControlMessage toMessage() const override;
    std::string toString() const override;

private:
    template<typename T> void addControls(T&& arg) {
        controls.push_back(std::move(arg));
    }

    template<typename T, typename... Args> void addControls(T&& arg, Args&&... args) {
        controls.push_back(std::move(arg));
        addControls(std::forward<Args>(args)...);
    }
};
}  // namespace Foundation
}  // namespace xiaohu_robot
#endif