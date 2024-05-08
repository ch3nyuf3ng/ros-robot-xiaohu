#include "xiaohu_robot/Nodes/TaskControllerNode.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/VelocityCommand.hpp"
#include "xiaohu_robot/GeneralTaskMessage.h"
#include <stdexcept>
#include <utility>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;

    ros::init(argc, argv, "task_ctrl_node");
    TaskControllerNode taskControllerNode{TaskControllerNode::Config{10_s}};
    taskControllerNode.run();

    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
TaskControllerNode::Config::Config(
    Duration initialPositionCalibrationTime,
    std::string baseStationName,
    std::string velocityCommandTopic,
    std::string objectDetectionControlTopic,
    std::string navigationWaypointTopic,
    std::string objectGrabbingCoordinateTopic,
    std::string manipulatorControlTopic,
    std::string detectedObjectCoordinatesTopic,
    std::string navigationResultTopic,
    std::string objectGrabbingResultTopic,
    std::string legacyGeneralTasksTopic,
    std::string speakTextTopic,
    std::string moveBaseTopic,
    NodeBasicConfig nodeBasicConfig
):
    initialPositionCalibrationTime{std::move(initialPositionCalibrationTime)},
    baseStationName{std::move(baseStationName)},
    velocityCommandTopic{std::move(velocityCommandTopic)},
    objectDetectionControlTopic{std::move(objectDetectionControlTopic)},
    navigationWaypointTopic{std::move(navigationWaypointTopic)},
    objectGrabbingCoordinateTopic{std::move(objectGrabbingCoordinateTopic)},
    manipulatorControlTopic{std::move(manipulatorControlTopic)},
    detectedObjectCoordinatesTopic{std::move(detectedObjectCoordinatesTopic)},
    navigationResultTopic{std::move(navigationResultTopic)},
    objectGrabbingResultTopic{std::move(objectGrabbingResultTopic)},
    legacyGeneralTasksTopic{std::move(legacyGeneralTasksTopic)},
    speakTextTopic{std::move(speakTextTopic)},
    moveBaseTopic{std::move(moveBaseTopic)} {
    sleep(1);
}

TaskControllerNode::TaskControllerNode(Config configs):
    configs{std::move(configs)},
    nodeHandle{},
    nodeTiming{this->configs.nodeBasicConfig},
    currentTaskState{TaskState::CalibratingInitialPosition},
    previousTaskState{TaskState::ReadyToPerformTasks},
    velocityCommandMessagePublisher{nodeHandle.advertise<VelocityCommandMessage>(
        this->configs.velocityCommandTopic, this->configs.nodeBasicConfig.messageBufferSize
    )},
    objectDetectionControlMessagePublisher{nodeHandle.advertise<StringMessage>(
        this->configs.objectDetectionControlTopic, this->configs.nodeBasicConfig.messageBufferSize
    )},
    navigationWaypointMessagePublisher{nodeHandle.advertise<StringMessage>(
        this->configs.navigationWaypointTopic, this->configs.nodeBasicConfig.messageBufferSize
    )},
    objectGrabbingCoodinateMessagePublisher{nodeHandle.advertise<CoordinateMessage>(
        this->configs.objectGrabbingCoordinateTopic, this->configs.nodeBasicConfig.messageBufferSize
    )},
    manipulatiorControlMessagePublisher{nodeHandle.advertise<ManipulatorControlMessage>(
        this->configs.manipulatorControlTopic, this->configs.nodeBasicConfig.messageBufferSize
    )},
    speakTextMessagePublisher{nodeHandle.advertise<StringMessage>(
        this->configs.speakTextTopic, this->configs.nodeBasicConfig.messageBufferSize
    )},
    detectedObjectCoordinatesMessageSubscriber{nodeHandle.subscribe<ObjectDetectionResultMessasgePointer>(
        this->configs.detectedObjectCoordinatesTopic,
        this->configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedObjectDetectionResult,
        this
    )},
    navigationResultMessageSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        this->configs.navigationResultTopic,
        this->configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedNavigationResult,
        this
    )},
    objectGrabResultMessageSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        this->configs.objectGrabbingResultTopic,
        this->configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedObjectGrabResult,
        this
    )},
    legacyGeneralTasksMessageSubscriber{nodeHandle.subscribe<GeneralTaskMessage>(
        this->configs.legacyGeneralTasksTopic,
        this->configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedLegacyGeneralTaskRequest,
        this
    )},
    navigationClient{this->configs.moveBaseTopic} {
    displayInitializationResult();
}

void TaskControllerNode::run() {
    ros::Rate loop_rate{configs.nodeBasicConfig.loopFrequency};
    while (ros::ok()) {
        switch (getCurrentTaskState()) {
        case TaskState::CalibratingInitialPosition:
            startInitialPositionCalibration();
            break;
        case TaskState::ReadyToPerformTasks:
            readyToPerformTasks();
            break;
        case TaskState::GoingToPharmacy:
            goToPharmacy();
            break;
        case TaskState::DetectingMedicine:
            detectMedicine();
            break;
        case TaskState::PerformingObjectGrab:
            performingObjectGrab();
            break;
        case TaskState::GoingToPatient:
            goToPatient();
            break;
        case TaskState::DeliveringMedicine:
            deliverMedicine();
            break;
        case TaskState::MovingBackward:
            stepBackward();
            break;
        case TaskState::RetractingTheArm:
            retractManipulaor();
            break;
        case TaskState::HaveFinishedPreviousTask:
            haveFinishedPreviousTask();
            break;
        case TaskState::GoingToBaseStation:
            goToBaseStation();
            break;
        case TaskState::WaypointUnreachable:
            waypointUnreachable();
            break;
        case TaskState::MeasuringTemperature:
            measuringTemperature();
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

LegacyGeneralTask& TaskControllerNode::getCurrentTask() {
    return legacyGeneralTasks.front();
}

TaskControllerNode::TaskState TaskControllerNode::getCurrentTaskState() const {
    return currentTaskState;
}

TaskControllerNode::TaskState TaskControllerNode::getPreviousTaskState() const {
    return previousTaskState;
}

void TaskControllerNode::setCurrentTaskState(TaskState nextState) {
    nodeTiming.resetTiming();
    auto oldState = currentTaskState;
    currentTaskState = nextState;
    setPreviousTaskState(oldState);
    ROS_INFO("State: %s -> %s", toString(oldState).c_str(), toString(currentTaskState).c_str());
}

void TaskControllerNode::setPreviousTaskState(TaskState previousState) {
    previousTaskState = previousState;
}

void TaskControllerNode::startInitialPositionCalibration() {
    if (nodeTiming.getTiming() == 0_s) {
        ROS_INFO(
            "Please estimate initial position in %f seconds.", configs.initialPositionCalibrationTime.getBaseUnitValue()
        );
        delegateSpeaking("请确认机器人初始位置位于充电基站。");
    }
    else if (nodeTiming.getTiming() > configs.initialPositionCalibrationTime) {
        setCurrentTaskState(TaskState::ReadyToPerformTasks);
        return;
    }
    nodeTiming.incrementTiming();
}

void TaskControllerNode::readyToPerformTasks() {
    if (!legacyGeneralTasks.empty()) {
        if (getCurrentTask().getTaskType() == TaskType::MedicineDelivery)
            setCurrentTaskState(TaskState::GoingToPharmacy);
        else if (getCurrentTask().getTaskType() == TaskType::Inspection)
            setCurrentTaskState(TaskState::GoingToPatient);
        else
            throw std::runtime_error{""};
        return;
    }
    nodeTiming.incrementTiming();
}

void TaskControllerNode::goToPharmacy() {
    if (nodeTiming.getTiming() == 0_s)
        delegateNavigatingToWaypoint(getCurrentTask().pharmacy.c_str());
    nodeTiming.incrementTiming();
}

void TaskControllerNode::detectMedicine() {
    if (nodeTiming.getTiming() == 0_s)
        delegateChangingRobotBehavior(ObjectDetectionControl::Start);
    nodeTiming.incrementTiming();
}

void TaskControllerNode::performingObjectGrab() {
    if (nodeTiming.getTiming() == 0_s)
        delegateObjectGrabbing(getCurrentTask().medicinePosition);
    nodeTiming.incrementTiming();
}

void TaskControllerNode::goToPatient() {
    if (nodeTiming.getTiming() == 0_s) {
        if (getCurrentTask().type == TaskType::MedicineDelivery
            || getCurrentTask().getTaskType() == TaskType::Inspection)
            delegateNavigatingToWaypoint(getCurrentTask().patient);
        else
            throw std::runtime_error{""};
    }
    nodeTiming.incrementTiming();
}

void TaskControllerNode::deliverMedicine() {
    static bool haveDropped{false};
    if (nodeTiming.getTiming() == 0_s) {
        delegateSpeaking(getCurrentTask().prescription);
    }
    else if (nodeTiming.getTiming() > 10_s && !haveDropped) {
        delegateControlingRobotManipulator(GripperControl{15_cm});
        haveDropped = true;
    }
    else if (nodeTiming.getTiming() > 20_s) {
        setCurrentTaskState(TaskState::MovingBackward);
        haveDropped = false;
        return;
    }
    nodeTiming.incrementTiming();
}

void TaskControllerNode::stepBackward() {
    if (nodeTiming.getTiming() == 0_s)
        delegateControlingRobotVelocity(-0.1_m_per_s);
    else if (nodeTiming.getTiming() > 5_s) {
        delegateControlingRobotVelocity(0_m_per_s);
        setCurrentTaskState(TaskState::RetractingTheArm);
        return;
    }
    nodeTiming.incrementTiming();
}

void TaskControllerNode::retractManipulaor() {
    if (nodeTiming.getTiming() == 0_s)
        delegateControlingRobotManipulator(ArmControl{0_cm});
    else if (nodeTiming.getTiming() > 5_s) {
        setCurrentTaskState(TaskState::HaveFinishedPreviousTask);
        return;
    }
    nodeTiming.incrementTiming();
}

void TaskControllerNode::haveFinishedPreviousTask() {
    if (getCurrentTask().getTaskType() == TaskType::MedicineDelivery) {
        ROS_INFO(
            "Have finished the previous medicine-delivery task (from %s to %s).",
            getCurrentTask().pharmacy.c_str(),
            getCurrentTask().patient.c_str()
        );
    }
    else if (getCurrentTask().getTaskType() == TaskType::Inspection) {
        ROS_INFO("Have finished the previous inspection task (%s).", getCurrentTask().patient.c_str());
    }
    else {
        throw std::runtime_error{""};
    }
    legacyGeneralTasks.pop_front();
    showTasks();
    if (!legacyGeneralTasks.empty()) {
        startNextTask();
    }
    else {
        setCurrentTaskState(TaskState::GoingToBaseStation);
    }
}

void TaskControllerNode::goToBaseStation() {
    static bool isCancelled{false};
    static Duration cancellationTime{0_s};
    if (nodeTiming.getTiming() == 0_s)
        delegateNavigatingToWaypoint(configs.baseStationName);
    if (!legacyGeneralTasks.empty() && !isCancelled) {
        navigationClient.cancelAllGoals();
        cancellationTime = nodeTiming.getTiming();
        isCancelled = true;
    }
    else if (isCancelled && nodeTiming.getTiming() - cancellationTime > 2_s) {
        startNextTask();
        isCancelled = false;
        return;
    }
    nodeTiming.incrementTiming();
}

void TaskControllerNode::waypointUnreachable() {
    if (nodeTiming.getTiming() == 0_s) {
        delegateSpeaking("无法前往下一处航点，请尝试移除周围的障碍物，机器人会在 20 秒内重试。");
    }
    else if (nodeTiming.getTiming() > 20_s) {
        setCurrentTaskState(getPreviousTaskState());
        return;
    }
    nodeTiming.incrementTiming();
}

void TaskControllerNode::measuringTemperature() {
    static bool haveSpoken = false;
    if (nodeTiming.getTiming() == 0_s) {
        delegateSpeaking("测温中。");
        ROS_INFO("Start measuring temperature.");
    }
    else if (nodeTiming.getTiming() > 5_s && !haveSpoken) {
        delegateSpeaking("测温完成。体温正常。");
        haveSpoken = true;
    }
    else if (nodeTiming.getTiming() >= 10_s) {
        haveSpoken = false;
        setCurrentTaskState(TaskState::HaveFinishedPreviousTask);
        return;
    }
    // showTiming();
    nodeTiming.incrementTiming();
}

void TaskControllerNode::showTasks() const {
    if (legacyGeneralTasks.empty())
        ROS_INFO("No task to do.");
    else {
        ROS_INFO("Task to do count: %zd", legacyGeneralTasks.size());
        std::size_t task_index{1};
        for (auto const& task : legacyGeneralTasks) {
            ROS_INFO("Task %zd: %s -> %s", task_index, task.pharmacy.c_str(), task.patient.c_str());
        }
    }
}

void TaskControllerNode::showTiming() const {
    ROS_INFO("Current timing: %s", nodeTiming.getTiming().toString().c_str());
}

void TaskControllerNode::displayInitializationResult() const {
    ROS_INFO(
        "Controller initialized.\n"
        "baseStationName: %s\n"
        "stateCheckFrequency: %s\n"
        "initialPositionCalibrationTime: %s\n"
        "velocityControlMessagePublisher: %s\n"
        "behaviorControlMessagePublisher: %s\n"
        "navigationWaypointMessagePublisher: %s\n"
        "objectToGrabCoodinateMessagePublisher: %s\n"
        "manipulatiorControlMessagePublisher: %s\n"
        "detectedObjectCoordinatesMessageSubscriber: %s\n"
        "navigationResultMessageSubscriber: %s\n"
        "objectGrabResultMessageSubscriber: %s\n"
        "objectMoveTasksMessageSubscriber: %s\n"
        "taskStateControlMessageSubscriber: %s\n",
        configs.baseStationName.c_str(),
        configs.nodeBasicConfig.loopFrequency.toString().c_str(),
        configs.initialPositionCalibrationTime.toString().c_str(),
        velocityCommandMessagePublisher.getTopic().c_str(),
        manipulatiorControlMessagePublisher.getTopic().c_str(),
        navigationWaypointMessagePublisher.getTopic().c_str(),
        objectGrabbingCoodinateMessagePublisher.getTopic().c_str(),
        manipulatiorControlMessagePublisher.getTopic().c_str(),
        detectedObjectCoordinatesMessageSubscriber.getTopic().c_str(),
        navigationResultMessageSubscriber.getTopic().c_str(),
        objectGrabResultMessageSubscriber.getTopic().c_str(),
        legacyGeneralTasksMessageSubscriber.getTopic().c_str(),
        taskStateControlMessageSubscriber.getTopic().c_str()
    );
}

void TaskControllerNode::startNextTask() {
    if (getCurrentTask().getTaskType() == TaskType::MedicineDelivery) {
        setCurrentTaskState(TaskState::GoingToPharmacy);
    }
    else if (getCurrentTask().getTaskType() == TaskType::Inspection) {
        setCurrentTaskState(TaskState::GoingToPatient);
    }
    else {
        throw std::runtime_error{""};
    }
}

void TaskControllerNode::whenReceivedObjectDetectionResult(ObjectDetectionResultMessasgePointer coordinates_ptr) {
    if (getCurrentTaskState() == TaskState::DetectingMedicine) {
        displayDetectedObjects(coordinates_ptr);

        constexpr std::size_t nearest_object_index{0};
        Coordinate coord{
            coordinates_ptr->x[nearest_object_index],
            coordinates_ptr->y[nearest_object_index],
            coordinates_ptr->z[nearest_object_index]
        };
        getCurrentTask().medicinePosition = coord;
        ROS_INFO("temp coordinate:\n%s", coord.toString().c_str());
        ROS_INFO("Current task coordinate:\n%s", getCurrentTask().medicinePosition.toString().c_str());
        delegateChangingRobotBehavior(ObjectDetectionControl::Stop);
        setCurrentTaskState(TaskState::PerformingObjectGrab);
    }
    else {
        ROS_DEBUG("Discard object detection result because of unmatching state.");
    }
}

void TaskControllerNode::whenReceivedNavigationResult(StringMessagePointer message_ptr) {
    if (message_ptr->data == "done")
        if (getCurrentTaskState() == TaskState::GoingToPharmacy) {
            ROS_INFO(
                "Arrived pharmacy: %s (Spent time: %s)",
                getCurrentTask().pharmacy.c_str(),
                nodeTiming.getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::DetectingMedicine);
        }
        else if (getCurrentTaskState() == TaskState::GoingToPatient) {
            ROS_INFO(
                "Arrived patient: %s (Spent time: %s)",
                getCurrentTask().patient.c_str(),
                nodeTiming.getTiming().toString().c_str()
            );
            if (getCurrentTask().getTaskType() == TaskType::MedicineDelivery)
                setCurrentTaskState(TaskState::DeliveringMedicine);
            else if (getCurrentTask().getTaskType() == TaskType::Inspection)
                setCurrentTaskState(TaskState::MeasuringTemperature);
            else
                throw std::runtime_error{""};
        }
        else if (getCurrentTaskState() == TaskState::GoingToBaseStation) {
            ROS_INFO(
                "Arrived base station: %s (Spent time: %s)",
                configs.baseStationName.c_str(),
                nodeTiming.getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::ReadyToPerformTasks);
        }
        else {
            ROS_DEBUG("Discard navigation result because of unmatching state.");
        }
    else if (message_ptr->data == "failure") {
        if (getCurrentTaskState() == TaskState::GoingToPharmacy) {
            ROS_INFO(
                "Failed to arrive pharmacy: %s (Spent time: %s)",
                getCurrentTask().pharmacy.c_str(),
                nodeTiming.getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::WaypointUnreachable);
        }
        else if (getCurrentTaskState() == TaskState::GoingToPatient) {
            ROS_INFO(
                "Failed to arrive patient: %s (Spent time: %s)",
                getCurrentTask().patient.c_str(),
                nodeTiming.getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::WaypointUnreachable);
        }
        else if (getCurrentTaskState() == TaskState::GoingToBaseStation && legacyGeneralTasks.empty()) {
            ROS_INFO(
                "Failed to arrive base station: %s (Spent time: %s)",
                configs.baseStationName.c_str(),
                nodeTiming.getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::WaypointUnreachable);
        }
        else {
            ROS_DEBUG("Discard navigation result because of unmatching state.");
        }
    }
    else {
        ROS_DEBUG("Discard navigation result: %s", message_ptr->data.c_str());
    }
}

void TaskControllerNode::whenReceivedObjectGrabResult(StringMessagePointer message_ptr) {
    if (message_ptr->data == "done") {
        if (getCurrentTaskState() == TaskState::PerformingObjectGrab) {
            ROS_INFO("Grab object done!");
            setCurrentTaskState(TaskState::GoingToPatient);
        }
        else {
            ROS_DEBUG("Discard object grab result because of unmatching state.");
        }
    }
    else {
        ROS_DEBUG("Discarded object grab result: %s", message_ptr->data.c_str());
    }
}

void TaskControllerNode::whenReceivedLegacyGeneralTaskRequest(GeneralTaskMessagePointer message) {
    legacyGeneralTasks.emplace_back(LegacyGeneralTask{message});
    ROS_INFO("Added a task:\n%s", legacyGeneralTasks.front().toString().c_str());
}

void TaskControllerNode::whenReceivedStateControlCommand(StringMessagePointer message_ptr) {
    if (message_ptr->data == "clear") {
        legacyGeneralTasks.clear();
        ROS_INFO("Cancelled all tasks.");
        setCurrentTaskState(TaskState::RetractingTheArm);
    }
}

void TaskControllerNode::delegateControlingRobotVelocity(LinearSpeed target) {
    ROS_INFO(
        "Notified subscriber of %s to set velocity to %s.",
        velocityCommandMessagePublisher.getTopic().c_str(),
        target.toString().c_str()
    );
    VelocityCommand velocityCommand{target, 0_m_per_s, 0_deg_per_s};
    velocityCommandMessagePublisher.publish(velocityCommand.toMessage());
}

void TaskControllerNode::delegateChangingRobotBehavior(ObjectDetectionControl behavior) {
    ROS_INFO(
        "Notified subscriber of %s to change behavior to %s.",
        objectDetectionControlMessagePublisher.getTopic().c_str(),
        toString(behavior).c_str()
    );
    objectDetectionControlMessagePublisher.publish(createObjectDetectionControlMessage(behavior));
}

void TaskControllerNode::delegateNavigatingToWaypoint(std::string waypointName) {
    ROS_INFO(
        "Notified subscriber of %s to navigate to %s.",
        navigationWaypointMessagePublisher.getTopic().c_str(),
        waypointName.c_str()
    );
    navigationWaypointMessagePublisher.publish(createWaypointMessage(std::move(waypointName)));
}

void TaskControllerNode::delegateControlingRobotManipulator(ManipulatorControl const& plan) {
    ROS_INFO(
        "Notified subscriber of %s to set manipulator state to %s.",
        manipulatiorControlMessagePublisher.getTopic().c_str(),
        plan.toString().c_str()
    );
    manipulatiorControlMessagePublisher.publish(plan.toMessage());
}

void TaskControllerNode::delegateObjectGrabbing(Coordinate coordinate) {
    ROS_INFO(
        "Notified subscriber of %s to grab object at %s.",
        objectGrabbingCoodinateMessagePublisher.getTopic().c_str(),
        coordinate.toString().c_str()
    );
    objectGrabbingCoodinateMessagePublisher.publish(coordinate.toMessage());
}

void TaskControllerNode::delegateSpeaking(std::string content) {
    std::printf(
        "Notified subscriber of %s to speak %s.\n", speakTextMessagePublisher.getTopic().c_str(), content.c_str()
    );
    StringMessage message;
    message.data = content;
    speakTextMessagePublisher.publish(message);
}

StringMessage TaskControllerNode::createWaypointMessage(std::string name) {
    StringMessage message{};
    message.data = std::move(name);
    return message;
}

StringMessage TaskControllerNode::createObjectDetectionControlMessage(ObjectDetectionControl behavior) {
    StringMessage message{};
    switch (behavior) {
    case ObjectDetectionControl::Start:
        message.data = "object_detect start";
        break;
    case ObjectDetectionControl::Stop:
        message.data = "object_detect stop";
        break;
    }
    return message;
}

void TaskControllerNode::displayDetectedObjects(ObjectDetectionResultMessasgePointer coordinates_ptr) {
    std::size_t const total_amount{coordinates_ptr->name.size()};
    ROS_INFO("Total amount of objects: %zd", total_amount);
    for (std::size_t object_id{0}; object_id < total_amount; ++object_id) {
        ROS_INFO(
            "object %s's coordinate: (%.2f, %.2f, %.2f)%s",
            coordinates_ptr->name[object_id].c_str(),
            coordinates_ptr->x[object_id],
            coordinates_ptr->y[object_id],
            coordinates_ptr->z[object_id],
            object_id == 0 ? " which is nearest and will be picked" : ""
        );
    }
}

std::string TaskControllerNode::toString(ObjectDetectionControl behavior) {
    switch (behavior) {
    case ObjectDetectionControl::Start:
        return "Start Object Detection";
    case ObjectDetectionControl::Stop:
        return "Stop Object Detection";
    }
}

std::string TaskControllerNode::toString(TaskState taskState) {
    switch (taskState) {
    case TaskState::CalibratingInitialPosition:
        return "Calibrating Initial Position";
    case TaskState::ReadyToPerformTasks:
        return "Ready To Perform Tasks";
    case TaskState::GoingToPharmacy:
        return "Going To Storage";
    case TaskState::DetectingMedicine:
        return "Detecting Objects";
    case TaskState::PerformingObjectGrab:
        return "Performing Object Grab";
    case TaskState::GoingToPatient:
        return "Going To Patient Place";
    case TaskState::DeliveringMedicine:
        return "Performing Object Drop-Off";
    case TaskState::MovingBackward:
        return "Stepping Backward";
    case TaskState::RetractingTheArm:
        return "Retracting The Arm";
    case TaskState::HaveFinishedPreviousTask:
        return "Have Finished Previous Task";
    case TaskState::GoingToBaseStation:
        return "Going To Base Station";
    case TaskState::WaypointUnreachable:
        return "Waypoint Unreachable";
    case TaskState::MeasuringTemperature:
        return "Measuring Temperature";
    }
}
}  // namespace Nodes
}  // namespace xiaohu_robot