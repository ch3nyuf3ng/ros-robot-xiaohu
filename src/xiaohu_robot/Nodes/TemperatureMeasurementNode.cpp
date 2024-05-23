#include "xiaohu_robot/Nodes/TemperatureMeasurementNode.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "xiaohu_robot/Foundation/InfraredTemperatureSensor.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <chrono>
#include <clocale>
#include <memory>
#include <thread>
#include <utility>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");
    ros::init(argc, argv, "temperature_measurement_node");
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
    bool useRealValue;
    if (nodeHandle.getParam("real_temperature", useRealValue) && useRealValue) {
        ROS_INFO("使用真实温度。");
        infraredTemperatureSensor = std::make_unique<InfraredTemperatureSensor>();
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