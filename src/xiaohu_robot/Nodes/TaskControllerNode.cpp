#include "xiaohu_robot/Nodes/TaskControllerNode.hpp"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/VelocityCommand.hpp"
#include <utility>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    ros::init(argc, argv, "task_ctrl_node");
    TaskControllerNodeConfigs taskControllerNodeConfig{
        CommonConfigs::baseStationName,
        CommonConfigs::velocityCommandTopic,
        CommonConfigs::objectDetectionControlTopic,
        CommonConfigs::navigationWaypointTopic,
        CommonConfigs::grabObjectCoodinateTopic,
        CommonConfigs::manipulatorControlTopic,
        CommonConfigs::detectedObjectCoordinatesTopic,
        CommonConfigs::navigationResultTopic,
        CommonConfigs::objectGrabbingResultTopic,
        CommonConfigs::objectMovingTasksTopic,
        CommonConfigs::taskStateControlTopic,
        CommonConfigs::speakTextTopic,
        CommonConfigs::messageBufferSize,
        CommonConfigs::nodeNamespace,
        CommonConfigs::stateCheckingFrequency,
        10_s,
        10_cm,
        true
    };
    TaskControllerNode taskControllerNode{taskControllerNodeConfig};
    taskControllerNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
TaskControllerNodeConfigs::TaskControllerNodeConfigs(
    std::string baseStationName,
    std::string velocityCommandTopic,
    std::string objectDetectionControlTopic,
    std::string navigationWaypointTopic,
    std::string objectGrabbingCoordinateTopic,
    std::string manipulatorControlTopic,
    std::string detectedObjectCoordinatesTopic,
    std::string navigationResultTopic,
    std::string objectGrabbingResultTopic,
    std::string objectMovingTasksTopic,
    std::string taskStateControlTopic,
    std::string speakTextTopic,
    std::size_t messageBufferSize,
    std::string nodeNamespace,
    Frequency stateCheckingFrequency,
    Duration initialPositionCalibrationTime,
    Length heightCompensation,
    bool hasManipulator
):
    baseStationName{std::move(baseStationName)},
    velocityCommandTopic{std::move(velocityCommandTopic)},
    objectDetectionControlTopic{std::move(objectDetectionControlTopic)},
    navigationWaypointTopic{std::move(navigationWaypointTopic)},
    objectGrabbingCoordinateTopic{std::move(objectGrabbingCoordinateTopic)},
    manipulatorControlTopic{std::move(manipulatorControlTopic)},
    detectedObjectCoordinatesTopic{std::move(detectedObjectCoordinatesTopic)},
    navigationResultTopic{std::move(navigationResultTopic)},
    objectGrabbingResultTopic{std::move(objectGrabbingResultTopic)},
    objectMovingTasksTopic{std::move(objectMovingTasksTopic)},
    taskStateControlTopic{std::move(taskStateControlTopic)},
    speakTextTopic{std::move(speakTextTopic)},
    messageBufferSize{messageBufferSize},
    nodeNamespace{std::move(nodeNamespace)},
    stateCheckingFrequency{std::move(stateCheckingFrequency)},
    initialPositionCalibrationTime{std::move(initialPositionCalibrationTime)},
    heightCompensation{std::move(heightCompensation)},
    hasManipulator{hasManipulator} {}

TaskControllerNode::TaskControllerNode(TaskControllerNodeConfigs configs):
    nodeHandle{configs.nodeNamespace},
    currentTaskState{TaskState::CalibratingInitialPosition},
    previousTaskState{TaskState::ReadyToPerformTasks},
    currentTiming{0_s},
    velocityCommandMessagePublisher{
        nodeHandle.advertise<VelocityCommandMessage>(configs.velocityCommandTopic, configs.messageBufferSize)
    },
    objectDetectionControlMessagePublisher{
        nodeHandle.advertise<StringMessage>(configs.objectDetectionControlTopic, configs.messageBufferSize)
    },
    navigationWaypointMessagePublisher{
        nodeHandle.advertise<StringMessage>(configs.navigationWaypointTopic, configs.messageBufferSize)
    },
    objectGrabbingCoodinateMessagePublisher{
        nodeHandle.advertise<CoordinateMessage>(configs.objectGrabbingCoordinateTopic, configs.messageBufferSize)
    },
    manipulatiorControlMessagePublisher{
        nodeHandle.advertise<ManipulatorControlMessage>(configs.manipulatorControlTopic, configs.messageBufferSize)
    },
    speakTextMessagePublisher{nodeHandle.advertise<StringMessage>(configs.speakTextTopic, configs.messageBufferSize)},
    detectedObjectCoordinatesMessageSubscriber{nodeHandle.subscribe<ObjectDetectionResultMessasgePointer>(
        configs.detectedObjectCoordinatesTopic,
        configs.messageBufferSize,
        &TaskControllerNode::whenReceivedObjectDetectionResult,
        this
    )},
    navigationResultMessageSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        configs.navigationResultTopic,
        configs.messageBufferSize,
        &TaskControllerNode::whenReceivedNavigationResult,
        this
    )},
    objectGrabResultMessageSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        configs.objectGrabbingResultTopic,
        configs.messageBufferSize,
        &TaskControllerNode::whenReceivedObjectGrabResult,
        this
    )},
    MedicineDeliveryTasksMessageSubscriber{nodeHandle.subscribe<ObjectMovingTaskMessagePointer>(
        configs.objectMovingTasksTopic,
        configs.messageBufferSize,
        &TaskControllerNode::whenReceivedMedicineDeliveryTaskRequest,
        this
    )},
    taskStateControlMessageSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        configs.taskStateControlTopic,
        configs.messageBufferSize,
        &TaskControllerNode::whenReceivedStateControlCommand,
        this
    )},
    configs{std::move(configs)} {
    displayInitializationResult();
}

void TaskControllerNode::run() {
    ros::Rate loop_rate{configs.stateCheckingFrequency};
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
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

TaskControllerNode::Task TaskControllerNode::getCurrentTask() const {
    return tasks.front();
}

TaskControllerNode::TaskState TaskControllerNode::getCurrentTaskState() const {
    return currentTaskState;
}

TaskControllerNode::TaskState TaskControllerNode::getPreviousTaskState() const {
    return previousTaskState;
}

Duration TaskControllerNode::getTiming() const {
    return currentTiming;
}

void TaskControllerNode::setCurrentTaskState(TaskState nextState) {
    resetTiming();
    auto oldState = currentTaskState;
    currentTaskState = nextState;
    setPreviousTaskState(oldState);
    ROS_INFO("State: %s -> %s", toString(oldState).c_str(), toString(currentTaskState).c_str());
}

void TaskControllerNode::setPreviousTaskState(TaskState previousState) {
    previousTaskState = previousState;
}

void TaskControllerNode::incrementTiming() {
    currentTiming += configs.stateCheckingFrequency.perCycleTime();
}

void TaskControllerNode::resetTiming() {
    currentTiming.set_value(0);
};

void TaskControllerNode::startInitialPositionCalibration() {
    if (getTiming() == 0_s)
        ROS_INFO(
            "Please estimate initial position in %f seconds.", configs.initialPositionCalibrationTime.getBaseUnitValue()
        );
    else if (getTiming() > configs.initialPositionCalibrationTime) {
        delegateSpeaking("测试文字转语音。");
        setCurrentTaskState(TaskState::ReadyToPerformTasks);
        return;
    }
    incrementTiming();
}

void TaskControllerNode::readyToPerformTasks() {
    if (!tasks.empty()) {
        if (getCurrentTask().storagePlaceName != "null")
            setCurrentTaskState(TaskState::GoingToPharmacy);
        else
            setCurrentTaskState(TaskState::GoingToPatient);
        return;
    }
    incrementTiming();
}

void TaskControllerNode::goToPharmacy() {
    if (getTiming() == 0_s)
        delegateNavigatingToWaypoint(getCurrentTask().storagePlaceName.c_str());
    incrementTiming();
}

void TaskControllerNode::detectMedicine() {
    if (getTiming() == 0_s)
        delegateChangingRobotBehavior(ObjectDetectionControl::Start);
    incrementTiming();
}

void TaskControllerNode::performingObjectGrab() {
    if (getTiming() == 0_s)
        delegateObjectGrabbing(getCurrentTask().objectStoragePosition);
    incrementTiming();
}

void TaskControllerNode::goToPatient() {
    if (getTiming() == 0_s) {
        if (getCurrentTask().storagePlaceName != "null")
            delegateNavigatingToWaypoint(getCurrentTask().dropOffPlaceName);
        else {
            delegateSpeaking("测温中");
        }
    }
    incrementTiming();
}

void TaskControllerNode::deliverMedicine() {
    if (getTiming() == 0_s)
        delegateControlingRobotManipulator(GripperControl{15_cm});
    else if (getTiming() > 5_s) {
        setCurrentTaskState(TaskState::MovingBackward);
        return;
    }
    incrementTiming();
}

void TaskControllerNode::stepBackward() {
    if (getTiming() == 0_s)
        delegateControlingRobotVelocity(-0.1_m_per_s);
    else if (getTiming() > 5_s) {
        delegateControlingRobotVelocity(0_m_per_s);
        setCurrentTaskState(TaskState::RetractingTheArm);
        return;
    }
    incrementTiming();
}

void TaskControllerNode::retractManipulaor() {
    if (getTiming() == 0_s)
        delegateControlingRobotManipulator(ArmControl{0_cm});
    else if (getTiming() > 5_s) {
        setCurrentTaskState(TaskState::HaveFinishedPreviousTask);
        return;
    }
    incrementTiming();
}

void TaskControllerNode::haveFinishedPreviousTask() {
    ROS_INFO(
        "Have finished the previous object-moving task (from %s to %s).",
        getCurrentTask().storagePlaceName.c_str(),
        getCurrentTask().dropOffPlaceName.c_str()
    );
    tasks.pop_front();
    showTasks();
    if (!tasks.empty())
        setCurrentTaskState(TaskState::GoingToPharmacy);
    else
        setCurrentTaskState(TaskState::GoingToBaseStation);
}

void TaskControllerNode::goToBaseStation() {
    if (getTiming() == 0_s)
        delegateNavigatingToWaypoint(configs.baseStationName);
    incrementTiming();
}

void TaskControllerNode::waypointUnreachable() {
    if (getTiming() == 0_s) {
        delegateSpeaking("无法前往下一处航点，请尝试移除周围的障碍物，机器人会在 20 秒内重试。");
    }
    else if (getTiming() > 20_s) {
        setCurrentTaskState(getPreviousTaskState());
        return;
    }
    incrementTiming();
}

void TaskControllerNode::showTasks() const {
    if (tasks.empty())
        ROS_INFO("No task to do.");
    else {
        ROS_INFO("Task to do count: %zd", tasks.size());
        std::size_t task_index{1};
        for (auto const& task : tasks) {
            ROS_INFO("Task %zd: %s -> %s", task_index, task.storagePlaceName.c_str(), task.dropOffPlaceName.c_str());
        }
    }
}

void TaskControllerNode::showTiming() const {
    ROS_INFO("Current timing: %s", getTiming().toString().c_str());
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
        configs.stateCheckingFrequency.toString().c_str(),
        configs.initialPositionCalibrationTime.toString().c_str(),
        velocityCommandMessagePublisher.getTopic().c_str(),
        manipulatiorControlMessagePublisher.getTopic().c_str(),
        navigationWaypointMessagePublisher.getTopic().c_str(),
        objectGrabbingCoodinateMessagePublisher.getTopic().c_str(),
        manipulatiorControlMessagePublisher.getTopic().c_str(),
        detectedObjectCoordinatesMessageSubscriber.getTopic().c_str(),
        navigationResultMessageSubscriber.getTopic().c_str(),
        objectGrabResultMessageSubscriber.getTopic().c_str(),
        MedicineDeliveryTasksMessageSubscriber.getTopic().c_str(),
        taskStateControlMessageSubscriber.getTopic().c_str()
    );
}

void TaskControllerNode::whenReceivedObjectDetectionResult(ObjectDetectionResultMessasgePointer coordinates_ptr) {
    if (getCurrentTaskState() == TaskState::DetectingMedicine) {
        displayDetectedObjects(coordinates_ptr);

        constexpr std::size_t nearest_object_index{0};
        getCurrentTask().objectStoragePosition
            = {coordinates_ptr->x[nearest_object_index],
               coordinates_ptr->y[nearest_object_index],
               coordinates_ptr->z[nearest_object_index]};

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
                getCurrentTask().storagePlaceName.c_str(),
                getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::DetectingMedicine);
        }
        else if (getCurrentTaskState() == TaskState::GoingToPatient) {
            ROS_INFO(
                "Arrived patient: %s (Spent time: %s)",
                getCurrentTask().dropOffPlaceName.c_str(),
                getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::DeliveringMedicine);
        }
        else if (getCurrentTaskState() == TaskState::GoingToBaseStation) {
            ROS_INFO(
                "Arrived base station: %s (Spent time: %s)",
                configs.baseStationName.c_str(),
                getTiming().toString().c_str()
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
                getCurrentTask().storagePlaceName.c_str(),
                getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::WaypointUnreachable);
        }
        else if (getCurrentTaskState() == TaskState::GoingToPatient) {
            ROS_INFO(
                "Failed to arrive patient: %s (Spent time: %s)",
                getCurrentTask().dropOffPlaceName.c_str(),
                getTiming().toString().c_str()
            );
            setCurrentTaskState(TaskState::WaypointUnreachable);
        }
        else if (getCurrentTaskState() == TaskState::GoingToBaseStation) {
            ROS_INFO(
                "Failed to arrive base station: %s (Spent time: %s)",
                configs.baseStationName.c_str(),
                getTiming().toString().c_str()
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

void TaskControllerNode::whenReceivedMedicineDeliveryTaskRequest(ObjectMovingTaskMessagePointer message_ptr) {
    tasks.emplace_back(Task{message_ptr->storage_place_name, message_ptr->drop_place_name});
    ROS_INFO(
        "Added task: move an object from %s to %s",
        message_ptr->storage_place_name.c_str(),
        message_ptr->drop_place_name.c_str()
    );
}

void TaskControllerNode::whenReceivedStateControlCommand(StringMessagePointer message_ptr) {
    if (message_ptr->data == "clear") {
        tasks.clear();
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
        "Notified subscriber of %s to speak %s.", speakTextMessagePublisher.getTopic().c_str(), content.c_str()
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
        return "Going To Drop-Off Place";
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
    }
}
}  // namespace Nodes
}  // namespace xiaohu_robot