#pragma once

#ifndef XIAOHU_ROBOT_NODES_TEMPERATURE_MEASUREMENT_NODE_HPP
#define XIAOHU_ROBOT_NODES_TEMPERATURE_MEASUREMENT_NODE_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/InfraredTemperatureSensor.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <random>

namespace xiaohu_robot {
inline namespace Nodes {
class TemperatureMeasurementNode final: public Runnable {
public:
    struct Configs final {
        std::string temperatureMeasurementRequestTopic{CommonConfigs::temperatureMeasurementRequestTopic};
        std::string temperatureMeasurementResultTopic{CommonConfigs::temperatureMeasurementResultTopic};
        NodeBasicConfigs nodeBasicConfigs;
    };

    struct RandomTemperature final {
        std::random_device randomDevice{};
        std::default_random_engine generator{randomDevice()};
        std::uniform_real_distribution<double> distribution{36.0, 40.0};

        double getValue();
    };

    TemperatureMeasurementNode(Configs);
    TemperatureMeasurementNode(TemperatureMeasurementNode const&) = delete;
    TemperatureMeasurementNode(TemperatureMeasurementNode&&) noexcept = delete;
    TemperatureMeasurementNode& operator=(TemperatureMeasurementNode const&) = delete;
    TemperatureMeasurementNode& operator=(TemperatureMeasurementNode&&) = delete;
    ~TemperatureMeasurementNode();
    void run() override;

private:
    RandomTemperature randomTemperature;
    NodeHandle nodeHandle;
    Subscriber temperatureMeasurementRequestSubscriber;
    Publisher temperatureMeasurementResultPublisher;
    Configs configs;
    std::unique_ptr<InfraredTemperatureSensor> infraredTemperatureSensor;

    void whenReceivedtemperatureMeasurementRequest(EmptyMessage::ConstPtr const&);
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif