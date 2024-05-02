#pragma once

#include "xiaohu_robot/Foundation/Task.hpp"
#ifndef XIAOHU_MOVE_OBJECT_NODE_HPP
#define XIAOHU_MOVE_OBJECT_NODE_HPP
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <cstddef>
#include <deque>
#include <string>

namespace xiaohu_robot {
inline namespace Nodes {
struct TaskControllerNodeConfigs final{
    std::string baseStationName;
    std::string velocityCommandTopic;
    std::string objectDetectionControlTopic;
    std::string navigationWaypointTopic;
    std::string objectGrabbingCoordinateTopic;
    std::string manipulatorControlTopic;
    std::string detectedObjectCoordinatesTopic;
    std::string navigationResultTopic;
    std::string objectGrabbingResultTopic;
    std::string legacyTasksTopic;
    std::string legacyGeneralTasksTopic;
    std::string taskStateControlTopic;
    std::string speakTextTopic;
    std::size_t messageBufferSize;
    std::string nodeNamespace;
    Frequency stateCheckingFrequency;
    Duration initialPositionCalibrationTime;
    Length heightCompensation;
    bool hasManipulator;

    TaskControllerNodeConfigs(
        std::string baseStationName,
        std::string velocityCommandTopic,
        std::string objectDetectionControlTopic,
        std::string navigationWaypointTopic,
        std::string objectGrabbingCoordinateTopic,
        std::string manipulatorControlTopic,
        std::string detectedObjectCoordinatesTopic,
        std::string navigationResultTopic,
        std::string objectGrabbingResultTopic,
        std::string legacyTasksTopic,
        std::string legacyGeneralTasksTopic,
        std::string taskStateControlTopic,
        std::string speakTextTopic,
        std::size_t messageBufferSize,
        std::string nodeNamespace,
        Frequency stateCheckingFrequency,
        Duration initialPositionCalibrationTime,
        Length heightCompensation,
        bool hasManipulator = true
    );
};

class TaskControllerNode final {
public:
    TaskControllerNode(TaskControllerNodeConfigs);
    void run();

private:
    enum class TaskState {
        CalibratingInitialPosition,
        ReadyToPerformTasks,
        GoingToPharmacy,
        DetectingMedicine,
        PerformingObjectGrab,
        GoingToPatient,
        MeasuringTemperature,
        DeliveringMedicine,
        MovingBackward,
        RetractingTheArm,
        HaveFinishedPreviousTask,
        GoingToBaseStation,
        WaypointUnreachable
    };

    enum class ObjectDetectionControl {
        Start,
        Stop
    };

    struct Task {
        std::string pharmacy;
        std::string patient;
        Coordinate medicinePosition;
    };

    NodeHandle nodeHandle;
    TaskState currentTaskState;
    TaskState previousTaskState;
    Duration currentTiming;
    std::deque<LegacyGeneralTask> legacyGeneralTasks;

    MessagePublisher const velocityCommandMessagePublisher;
    MessagePublisher const objectDetectionControlMessagePublisher;
    MessagePublisher const navigationWaypointMessagePublisher;
    MessagePublisher const objectGrabbingCoodinateMessagePublisher;
    MessagePublisher const manipulatiorControlMessagePublisher;
    MessagePublisher const speakTextMessagePublisher;
    MessageSubscriber const detectedObjectCoordinatesMessageSubscriber;
    MessageSubscriber const navigationResultMessageSubscriber;
    MessageSubscriber const objectGrabResultMessageSubscriber;
    MessageSubscriber const legacyGeneralTasksMessageSubscriber;
    MessageSubscriber const taskStateControlMessageSubscriber;

    TaskControllerNodeConfigs configs;

    LegacyGeneralTask& getCurrentTask();
    TaskState getCurrentTaskState() const;
    TaskState getPreviousTaskState() const;
    Duration getTiming() const;
    void setCurrentTaskState(TaskState nextState);
    void setPreviousTaskState(TaskState previousState);
    void incrementTiming();
    void resetTiming();

    void startInitialPositionCalibration();
    void readyToPerformTasks();
    void goToPharmacy();
    void detectMedicine();
    void performingObjectGrab();
    void goToPatient();
    void deliverMedicine();
    void stepBackward();
    void retractManipulaor();
    void haveFinishedPreviousTask();
    void goToBaseStation();
    void waypointUnreachable();
    void measuringTemperature();

    void showTasks() const;
    void showTiming() const;
    void displayInitializationResult() const;

    void whenReceivedObjectDetectionResult(ObjectDetectionResultMessasgePointer coordinates_ptr);
    void whenReceivedNavigationResult(StringMessagePointer message_ptr);
    void whenReceivedObjectGrabResult(StringMessagePointer message_ptr);
    void whenReceivedLegacyGeneralTaskRequest(GeneralTaskMessagePointer message);
    void whenReceivedStateControlCommand(StringMessagePointer message_ptr);

    void delegateControlingRobotVelocity(LinearSpeed target);
    void delegateChangingRobotBehavior(ObjectDetectionControl target);
    void delegateNavigatingToWaypoint(std::string target);
    void delegateControlingRobotManipulator(ManipulatorControl const& target);
    void delegateObjectGrabbing(Coordinate coordinate);
    void delegateSpeaking(std::string content);

    static void displayDetectedObjects(ObjectDetectionResultMessasgePointer coordinates_ptr);
    static StringMessage createWaypointMessage(std::string name);
    static StringMessage createObjectDetectionControlMessage(ObjectDetectionControl behavior);
    static std::string toString(ObjectDetectionControl behavior);
    static std::string toString(TaskState taskState);
};
}  // namespace Nodes
}  // namespace xiaohu_robot
#endif