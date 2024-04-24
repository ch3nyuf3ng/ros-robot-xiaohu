#include "xiaohu_robot/Controller.hpp"
#include <sstream>

using namespace xiaohu_robot;

Controller::Coordinate::Coordinate(double x, double y, double z):
    x{x},
    y{y},
    z{z} {}

std::string Controller::Coordinate::toString() const {
    std::ostringstream oss;
    oss << "Coordinate{x: " << x << ", y:" << y << ", z: " << z << "}";
    return oss.str();
}

Controller::Controller(
    std::string baseStationName,
    std::string velocityControlTopic,
    std::string behaviorControlTopic,
    std::string navigationWaypointNameTopic,
    std::string objectToGrabCoodinateTopic,
    std::string manipulatorControlTopic,
    std::string detectedObjectCoordinatesTopic,
    std::string navigationResultTopic,
    std::string objectGrabbingResultTopic,
    std::string objectMovingTasksTopic,
    std::string taskStateControlTopic,
    std::size_t messageBufferSize,
    Measurement<UnitFrequency> stateCheckingFrequency,
    Measurement<UnitDuration> initialPositionCalibrationTime
):
    nodeHandle{},
    objectMoveTasks{},
    timing{0_s},
    taskState{TaskState::calibratingInitialPosition},
    manipulatorArmState{ManipulatorArmState::retracted},
    objectToGrabCoordinate{0, 0, 0},
    baseStationName{std::move(baseStationName)},
    stateCheckFrequency{std::move(stateCheckingFrequency)},
    initialPositionCalibrationTime{std::move(initialPositionCalibrationTime)},
    velocityControlMessagePublisher{
        nodeHandle.advertise<VelocityMessage>(std::move(velocityControlTopic), messageBufferSize)
    },
    behaviorControlMessagePublisher{
        nodeHandle.advertise<StringMessage>(std::move(behaviorControlTopic), messageBufferSize)
    },
    navigationWaypointMessagePublisher{
        nodeHandle.advertise<StringMessage>(std::move(navigationWaypointNameTopic), messageBufferSize)
    },
    objectToGrabCoodinateMessagePublisher{
        nodeHandle.advertise<CoordinateMessage>(std::move(objectToGrabCoodinateTopic), messageBufferSize)
    },
    manipulatiorControlMessagePublisher{
        nodeHandle.advertise<ManipulatorControlMessage>(std::move(manipulatorControlTopic), messageBufferSize)
    },
    detectedObjectCoordinatesMessageSubscriber{nodeHandle.subscribe<CoordinatesMessasgPointer>(
        std::move(detectedObjectCoordinatesTopic), messageBufferSize, &Controller::processObjectDetectionResult, this
    )},
    navigationResultMessageSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        std::move(navigationResultTopic), messageBufferSize, &Controller::processNavigationResult, this
    )},
    objectGrabResultMessageSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        std::move(objectGrabbingResultTopic), messageBufferSize, &Controller::processObjectGrabResult, this
    )},
    objectMoveTasksMessageSubscriber{nodeHandle.subscribe<ObjectMovingTaskMessagePointer>(
        std::move(objectMovingTasksTopic), messageBufferSize, &Controller::processObjectMovingTaskRequest, this
    )},
    taskStateControlMessageSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        std::move(taskStateControlTopic), messageBufferSize, &Controller::processStateControlCommand, this
    )} {
    displayInitializationResult();
    sleep(1.0);  // Waiting for the topic to be published/subscribed properly.
}

void Controller::run() {
    ros::Rate loop_rate{stateCheckFrequency};
    while (ros::ok()) {
        switch (getTaskState()) {
        case TaskState::calibratingInitialPosition:
            startInitialPositionCalibration();
            break;
        case TaskState::readyToPerformTasks:
            readyToPerformTasks();
            break;
        case TaskState::goingToStorage:
            goToStorage();
            break;
        case TaskState::detectingObjects:
            detectObjects();
            break;
        case TaskState::performingObjectGrab:
            performingObjectGrab();
            break;
        case TaskState::goingToDropOffPlace:
            goToDropOffPlace();
            break;
        case TaskState::performingObjectDropOff:
            dropOffObject();
            break;
        case TaskState::steppingBackward:
            stepBackward();
            break;
        case TaskState::retractingTheArm:
            retractManipulaor();
            break;
        case TaskState::haveFinishedPreviousTask:
            haveFinishedPreviousTask();
            break;
        case TaskState::goingToBaseStation:
            goToBaseStation();
            break;
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

Controller::MoveTask Controller::getCurrentObjectMovingTask() const {
    return objectMoveTasks.front();
}

Controller::TaskState Controller::getTaskState() const {
    return taskState;
}

Controller::ManipulatorArmState Controller::getManipulatorArmState() const {
    return manipulatorArmState;
}

Controller::Duration Controller::getTiming() const {
    return timing;
}

Controller::Coordinate Controller::getObjectToGrabCoordinate() const {
    return objectToGrabCoordinate;
}

void Controller::setManipulatorArmState(ManipulatorArmState nextState) {
    manipulatorArmState = nextState;
}

void Controller::setTaskState(TaskState nextState) {
    resetTiming();
    auto oldState = taskState;
    taskState = nextState;
    ROS_INFO("State: %s -> %s", taskStateToString(oldState).c_str(), taskStateToString(taskState).c_str());
}

void Controller::setObjectToGrabCoordinate(Coordinate coordinate) {
    objectToGrabCoordinate = std::move(coordinate);
}

void Controller::incrementTiming() {
    timing += stateCheckFrequency.perCycleTime();
}

void Controller::resetTiming() {
    timing.set_value(0);
};

void Controller::startInitialPositionCalibration() {
    if (getTiming() == 0_s)
        ROS_INFO("Please estimate initial position in %f seconds.", initialPositionCalibrationTime.get_value());
    else if (getTiming() > initialPositionCalibrationTime) {
        setTaskState(TaskState::readyToPerformTasks);
        return;
    }
    incrementTiming();
}

void Controller::readyToPerformTasks() {
    if (!objectMoveTasks.empty()) {
        setTaskState(TaskState::goingToStorage);
        return;
    }
    incrementTiming();
}

void Controller::goToStorage() {
    if (getTiming() == 0_s)
        navigateToWaypoint(getCurrentObjectMovingTask().storageName.c_str());
    incrementTiming();
}

void Controller::detectObjects() {
    if (getTiming() == 0_s)
        changeRobotBehavior(Behavior::startObjectDetection);
    incrementTiming();
}

void Controller::performingObjectGrab() {
    if (getTiming() == 0_s)
        grabObject(getObjectToGrabCoordinate());
    incrementTiming();
}

void Controller::goToDropOffPlace() {
    if (getTiming() == 0_s)
        navigateToWaypoint(getCurrentObjectMovingTask().dropOffPlaceName);
    incrementTiming();
}

void Controller::dropOffObject() {
    if (getTiming() == 0_s)
        controlRobotManipulator(GripperControl{15_cm});
    else if (getTiming() > 5_s) {
        setTaskState(TaskState::steppingBackward);
        return;
    }
    incrementTiming();
}

void Controller::stepBackward() {
    if (getTiming() == 0_s)
        controlRobotVelocity(-0.1_m_per_s);
    else if (getTiming() > 5_s) {
        controlRobotVelocity(0_m_per_s);
        setTaskState(TaskState::retractingTheArm);
        return;
    }
    incrementTiming();
}

void Controller::retractManipulaor() {
    if (manipulatorArmState == ManipulatorArmState::extended) {
        if (getTiming() == 0_s)
            controlRobotManipulator(ArmControl{0_cm});
        else if (getTiming() > 5_s) {
            setManipulatorArmState(ManipulatorArmState::retracted);
            setTaskState(TaskState::haveFinishedPreviousTask);
            return;
        }
        incrementTiming();
    }
    else
        setTaskState(TaskState::haveFinishedPreviousTask);
}

void Controller::haveFinishedPreviousTask() {
    ROS_INFO(
        "Have finished the previous object-moving task (from %s to %s).",
        getCurrentObjectMovingTask().storageName.c_str(),
        getCurrentObjectMovingTask().dropOffPlaceName.c_str()
    );
    objectMoveTasks.pop_front();
    showTasks();
    if (!objectMoveTasks.empty())
        setTaskState(TaskState::goingToStorage);
    else
        setTaskState(TaskState::goingToBaseStation);
}

void Controller::goToBaseStation() {
    if (getTiming() == 0_s)
        navigateToWaypoint(baseStationName);
    incrementTiming();
}

void Controller::showTasks() const {
    if (objectMoveTasks.empty())
        ROS_INFO("No task to do.");
    else {
        ROS_INFO("Task to do count: %zd", objectMoveTasks.size());
        std::size_t task_index{1};
        for (auto const& task : objectMoveTasks) {
            ROS_INFO("Task %zd: %s -> %s", task_index, task.storageName.c_str(), task.dropOffPlaceName.c_str());
        }
    }
}

void Controller::showTiming() const {
    ROS_INFO("Current timing: %s", getTiming().toString().c_str());
}

void Controller::displayInitializationResult() const {
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
        baseStationName.c_str(),
        stateCheckFrequency.toString().c_str(),
        initialPositionCalibrationTime.toString().c_str(),
        velocityControlMessagePublisher.getTopic().c_str(),
        behaviorControlMessagePublisher.getTopic().c_str(),
        navigationWaypointMessagePublisher.getTopic().c_str(),
        objectToGrabCoodinateMessagePublisher.getTopic().c_str(),
        manipulatiorControlMessagePublisher.getTopic().c_str(),
        detectedObjectCoordinatesMessageSubscriber.getTopic().c_str(),
        navigationResultMessageSubscriber.getTopic().c_str(),
        objectGrabResultMessageSubscriber.getTopic().c_str(),
        objectMoveTasksMessageSubscriber.getTopic().c_str(),
        taskStateControlMessageSubscriber.getTopic().c_str()
    );
}

void Controller::processObjectDetectionResult(CoordinatesMessasgPointer coordinates_ptr) {
    if (getTaskState() == TaskState::detectingObjects) {
        displayDetectedObjects(coordinates_ptr);

        constexpr auto nearest_object_index{0};
        Coordinate coordinate{
            coordinates_ptr->x[nearest_object_index],
            coordinates_ptr->y[nearest_object_index],
            coordinates_ptr->z[nearest_object_index]
        };
        setObjectToGrabCoordinate(coordinate);

        changeRobotBehavior(Behavior::stopObjectDetection);
        setTaskState(TaskState::performingObjectGrab);
    }
    else {
        ROS_DEBUG("Discard object detection result because of unmatching state.");
    }
}

void Controller::processNavigationResult(StringMessagePointer message_ptr) {
    if (message_ptr->data == "done")
        if (getTaskState() == TaskState::goingToStorage) {
            ROS_INFO(
                "Arrived storage: %s (Spent time: %s)",
                getCurrentObjectMovingTask().storageName.c_str(),
                getTiming().toString().c_str()
            );
            setTaskState(TaskState::detectingObjects);
        }
        else if (getTaskState() == TaskState::goingToDropOffPlace) {
            ROS_INFO(
                "Arrived drop-off place: %s (Spent time: %s)",
                getCurrentObjectMovingTask().dropOffPlaceName.c_str(),
                getTiming().toString().c_str()
            );
            setTaskState(TaskState::performingObjectDropOff);
        }
        else if (getTaskState() == TaskState::goingToBaseStation) {
            ROS_INFO(
                "Arrived base station: %s (Spent time: %s)", baseStationName.c_str(), getTiming().toString().c_str()
            );
            setTaskState(TaskState::readyToPerformTasks);
        }
        else {
            ROS_DEBUG("Discard navigation result because of unmatching state.");
        }
    else {
        ROS_DEBUG("Discard navigation result: %s", message_ptr->data.c_str());
    }
}

void Controller::processObjectGrabResult(StringMessagePointer message_ptr) {
    if (message_ptr->data == "done") {
        if (getTaskState() == TaskState::performingObjectGrab) {
            ROS_INFO("Grab object done!");
            setManipulatorArmState(ManipulatorArmState::extended);
            setTaskState(TaskState::goingToDropOffPlace);
        }
        else {
            ROS_DEBUG("Discard object grab result because of unmatching state.");
        }
    }
    else {
        ROS_DEBUG("Discarded object grab result: %s", message_ptr->data.c_str());
    }
}

void Controller::processObjectMovingTaskRequest(ObjectMovingTaskMessagePointer message_ptr) {
    objectMoveTasks.emplace_back(MoveTask{message_ptr->storage_place_name, message_ptr->drop_place_name});
    ROS_INFO(
        "Added task: move an object from %s to %s",
        message_ptr->storage_place_name.c_str(),
        message_ptr->drop_place_name.c_str()
    );
}

void Controller::processStateControlCommand(StringMessagePointer message_ptr) {
    if (message_ptr->data == "clear") {
        objectMoveTasks.clear();
        ROS_INFO("Cancelled all tasks.");
        setTaskState(TaskState::retractingTheArm);
    }
}

void Controller::controlRobotVelocity(Velocity target) {
    ROS_INFO(
        "Notified subscriber of %s to set velocity to %s.",
        velocityControlMessagePublisher.getTopic().c_str(),
        target.toString().c_str()
    );
    velocityControlMessagePublisher.publish(createVelocityMessage(target));
}

void Controller::changeRobotBehavior(Behavior behavior) {
    ROS_INFO(
        "Notified subscriber of %s to change behavior to %s.",
        behaviorControlMessagePublisher.getTopic().c_str(),
        behaviorToString(behavior).c_str()
    );
    behaviorControlMessagePublisher.publish(createBehaviorMessage(behavior));
}

void Controller::navigateToWaypoint(String waypointName) {
    ROS_INFO(
        "Notified subscriber of %s to navigate to %s.",
        navigationWaypointMessagePublisher.getTopic().c_str(),
        waypointName.c_str()
    );
    navigationWaypointMessagePublisher.publish(createWaypointMessage(std::move(waypointName)));
}

void Controller::controlRobotManipulator(ManipulatorControl const& plan) {
    ROS_INFO(
        "Notified subscriber of %s to set manipulator state to %s.",
        navigationResultMessageSubscriber.getTopic().c_str(),
        plan.toString().c_str()
    );
    manipulatiorControlMessagePublisher.publish(plan.toMessage());
}

void Controller::grabObject(Coordinate coordinate) {
    ROS_INFO(
        "Notified subscriber of %s to grab object at %s.",
        objectToGrabCoodinateMessagePublisher.getTopic().c_str(),
        coordinate.toString().c_str()
    );
    objectToGrabCoodinateMessagePublisher.publish(createCoordinateMessage(coordinate));
}

Controller::VelocityMessage Controller::createVelocityMessage(Velocity velocity) {
    VelocityMessage message{};
    message.linear.x = velocity;
    return message;
}

Controller::StringMessage Controller::createWaypointMessage(String name) {
    StringMessage message{};
    message.data = std::move(name);
    return message;
}

Controller::StringMessage Controller::createBehaviorMessage(Behavior behavior) {
    StringMessage message{};
    switch (behavior) {
    case Behavior::startObjectDetection:
        message.data = "object_detect start";
        break;
    case Behavior::stopObjectDetection:
        message.data = "object_detect stop";
        break;
    }
    return message;
}

Controller::CoordinateMessage Controller::createCoordinateMessage(Coordinate coordinate) {
    CoordinateMessage message{};
    message.position.x = coordinate.x;
    message.position.y = coordinate.y;
    message.position.z = coordinate.z;
    return message;
}

void Controller::displayDetectedObjects(CoordinatesMessasgPointer coordinates_ptr) {
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

Controller::String Controller::behaviorToString(Behavior behavior) {
    switch (behavior) {
    case Behavior::startObjectDetection:
        return "Start Object Detection";
    case Behavior::stopObjectDetection:
        return "Stop Object Detection";
    }
}

Controller::String Controller::taskStateToString(TaskState taskState) {
    switch (taskState) {
    case TaskState::calibratingInitialPosition:
        return "Calibrating Initial Position";
    case TaskState::readyToPerformTasks:
        return "Ready To Perform Tasks";
    case TaskState::goingToStorage:
        return "Going To Storage";
    case TaskState::detectingObjects:
        return "Detecting Objects";
    case TaskState::performingObjectGrab:
        return "Performing Object Grab";
    case TaskState::goingToDropOffPlace:
        return "Going To Drop-Off Place";
    case TaskState::performingObjectDropOff:
        return "Performing Object Drop-Off";
    case TaskState::steppingBackward:
        return "Stepping Backward";
    case TaskState::retractingTheArm:
        return "Retracting The Arm";
    case TaskState::haveFinishedPreviousTask:
        return "Have Finished Previous Task";
    case TaskState::goingToBaseStation:
        return "Going To Base Station";
    }
}