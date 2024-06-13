#include "xiaohu_robot/Nodes/TemperatureMeasurementNode.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/InfraredTemperatureSensor.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <chrono>
#include <clocale>
#include <memory>
#include <stdexcept>
#include <thread>
#include <utility>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");
    ros::init(argc, argv, CommonConfigs::TemperatureMeasurementNodeName);
    TemperatureMeasurementNode temperatureMeasurementNode{{}};
    temperatureMeasurementNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
double TemperatureMeasurementNode::RandomTemperature::getValue() {
    return distribution(generator);
}

TemperatureMeasurementNode::TemperatureMeasurementNode(Configs configs):
    randomTemperature{},
    nodeHandle(configs.nodeBasicConfigs.nodeNamespace),
    temperatureMeasurementRequestSubscriber{nodeHandle.subscribe<EmptyMessage>(
        configs.temperatureMeasurementRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &TemperatureMeasurementNode::whenReceivedtemperatureMeasurementRequest,
        this
    )},
    temperatureMeasurementResultPublisher{nodeHandle.advertise<Float64Message>(
        configs.temperatureMeasurementResultTopic, configs.nodeBasicConfigs.messageBufferSize
    )},
    configs{std::move(configs)} {
    bool useRealValue{false};
    if (nodeHandle.getParam(configs.isRealParameter, useRealValue) && useRealValue) {
        ROS_INFO("使用真实温度。");
        try {
            infraredTemperatureSensor = std::make_unique<InfraredTemperatureSensorUart>();
        } catch (std::runtime_error const& e) {
            std::cout << e.what() << std::endl;
            std::cout << "使用 UART 协议通信失败，尝试使用 I2C 协议。" << std::endl;
            infraredTemperatureSensor = std::make_unique<InfraredTemperatureSensorI2c>();
        }
    } else {
        ROS_INFO("使用模拟温度。");
    }
    std::cout << "测温节点已启动。" << std::endl;
}

TemperatureMeasurementNode::~TemperatureMeasurementNode() {
    std::cout << "测温节点已退出。" << std::endl;
}

void TemperatureMeasurementNode::run() {
    ros::spin();
}

void TemperatureMeasurementNode::whenReceivedtemperatureMeasurementRequest(EmptyMessage::ConstPtr const&) {
    Float64Message result;
    if (infraredTemperatureSensor) {
        result.data = infraredTemperatureSensor->measureTemperature().bodyTemperature;
    } else {
        result.data = randomTemperature.getValue();
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    ROS_INFO("已测量体温：%.2f 摄氏度。", result.data);
    temperatureMeasurementResultPublisher.publish<Float64Message>(result);
}
}  // namespace Nodes
}  // namespace xiaohu_robot