#pragma once

#ifndef XIAOHU_MOVE_OBJECT_NODE_HPP
#define XIAOHU_MOVE_OBJECT_NODE_HPP
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <deque>
#include <string>

namespace xiaohu_robot {
inline namespace Nodes {
class TaskControllerNode final: public Runnable {
public:
    struct Config final {
        Length heightCompensation;
        Duration initialPositionCalibrationTime;
        std::string baseStationName;
        std::string velocityCommandTopic;
        std::string objectDetectionControlTopic;
        std::string navigationWaypointTopic;
        std::string objectGrabbingCoordinateTopic;
        std::string manipulatorControlTopic;
        std::string detectedObjectCoordinatesTopic;
        std::string navigationResultTopic;
        std::string objectGrabbingResultTopic;
        std::string legacyGeneralTasksTopic;
        std::string speakTextTopic;
        std::string moveBaseTopic;
        NodeBasicConfig nodeBasicConfig;

        Config(
            Length heightCompensation,
            Duration initialPositionCalibrationTime,
            std::string baseStationName = CommonConfigs::baseStationName,
            std::string velocityCommandTopic = CommonConfigs::velocityCommandTopic,
            std::string objectDetectionControlTopic = CommonConfigs::objectDetectionControlTopic,
            std::string navigationWaypointTopic = CommonConfigs::navigationWaypointTopic,
            std::string objectGrabbingCoordinateTopic = CommonConfigs::objectGrabbingCoodinateTopic,
            std::string manipulatorControlTopic = CommonConfigs::manipulatorControlTopic,
            std::string detectedObjectCoordinatesTopic = CommonConfigs::detectedObjectCoordinatesTopic,
            std::string navigationResultTopic = CommonConfigs::navigationResultTopic,
            std::string objectGrabbingResultTopic = CommonConfigs::objectGrabbingResultTopic,
            std::string legacyGeneralTasksTopic = CommonConfigs::legacyGeneralTasksTopic,
            std::string speakTextTopic = CommonConfigs::speakTextTopic,
            std::string moveBaseTopic = CommonConfigs::moveBaseTopic,
            NodeBasicConfig nodeBasicConfig = NodeBasicConfig{}
        );
    };

    TaskControllerNode(Config);
    void run() override;

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

    Config configs;
    NodeHandle nodeHandle;
    NodeTiming nodeTiming;
    TaskState currentTaskState;
    TaskState previousTaskState;
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
    NavigationClient navigationClient;

    LegacyGeneralTask& getCurrentTask();
    TaskState getCurrentTaskState() const;
    TaskState getPreviousTaskState() const;
    void setCurrentTaskState(TaskState nextState);
    void setPreviousTaskState(TaskState previousState);

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

    void startNextTask();

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