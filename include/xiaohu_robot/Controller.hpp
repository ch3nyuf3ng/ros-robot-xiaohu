#pragma once

#include "xiaohu_robot/Foundation/CommonInterfaces.hpp"
#ifndef XIAOHU_MOVE_OBJECT_NODE_HPP
#define XIAOHU_MOVE_OBJECT_NODE_HPP
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/Twist.h"
#include "ros/node_handle.h"
#include "ros/publisher.h"
#include "ros/subscriber.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/String.h"
#include "wpb_home_behaviors/Coord.h"
#include "xiaohu_robot/MoveTaskMsg.h"
#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include <cstddef>
#include <deque>
#include <string>

namespace xiaohu_robot {
class Controller final {
public:
    Controller(
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
    );
    void run();

private:
    enum class TaskState {
        calibratingInitialPosition,
        readyToPerformTasks,
        goingToStorage,
        detectingObjects,
        performingObjectGrab,
        goingToDropOffPlace,
        performingObjectDropOff,
        steppingBackward,
        retractingTheArm,
        haveFinishedPreviousTask,
        goingToBaseStation
    };

    enum class ManipulatorArmState {
        retracted,
        extended
    };

    enum class Behavior {
        startObjectDetection,
        stopObjectDetection
    };

    struct MoveTask {
        std::string storageName;
        std::string dropOffPlaceName;
    };

    struct Coordinate: public Printable {
        double x, y, z;

        Coordinate(double x, double y, double z);
        std::string toString() const;
    };

    using StringMessagePointer = std_msgs::String::ConstPtr const&;
    using ObjectMovingTaskMessagePointer = xiaohu_robot::MoveTaskMsg::ConstPtr const&;
    using CoordinatesMessasgPointer = wpb_home_behaviors::CoordConstPtr const&;
    using VelocityMessage = geometry_msgs::Twist;
    using StringMessage = std_msgs::String;
    using CoordinateMessage = geometry_msgs::Pose;
    using ManipulatorControlMessage = sensor_msgs::JointState;
    using NodeHandle = ros::NodeHandle;
    using MessagePublisher = ros::Publisher;
    using MessageSubscriber = ros::Subscriber;
    using ObjectMovingTaskQueue = std::deque<MoveTask>;
    using Duration = Measurement<UnitDuration>;
    using Frequency = Measurement<UnitFrequency>;
    using Velocity = Measurement<UnitSpeed>;
    using String = std::string;

    NodeHandle nodeHandle;
    ObjectMovingTaskQueue objectMoveTasks;
    Duration timing;
    TaskState taskState;
    ManipulatorArmState manipulatorArmState;
    Coordinate objectToGrabCoordinate;

    String const baseStationName;
    Frequency const stateCheckFrequency;
    Duration const initialPositionCalibrationTime;
    MessagePublisher const velocityControlMessagePublisher;
    MessagePublisher const behaviorControlMessagePublisher;
    MessagePublisher const navigationWaypointMessagePublisher;
    MessagePublisher const objectToGrabCoodinateMessagePublisher;
    MessagePublisher const manipulatiorControlMessagePublisher;
    MessageSubscriber const detectedObjectCoordinatesMessageSubscriber;
    MessageSubscriber const navigationResultMessageSubscriber;
    MessageSubscriber const objectGrabResultMessageSubscriber;
    MessageSubscriber const objectMoveTasksMessageSubscriber;
    MessageSubscriber const taskStateControlMessageSubscriber;

    MoveTask getCurrentObjectMovingTask() const;
    TaskState getTaskState() const;
    ManipulatorArmState getManipulatorArmState() const;
    Duration getTiming() const;
    Coordinate getObjectToGrabCoordinate() const;
    void setTaskState(TaskState next_state);
    void setManipulatorArmState(ManipulatorArmState next_state);
    void setObjectToGrabCoordinate(Coordinate coordinate);
    void incrementTiming();
    void resetTiming();

    void startInitialPositionCalibration();
    void readyToPerformTasks();
    void goToStorage();
    void detectObjects();
    void performingObjectGrab();
    void goToDropOffPlace();
    void dropOffObject();
    void stepBackward();
    void retractManipulaor();
    void haveFinishedPreviousTask();
    void goToBaseStation();
    
    void showTasks() const;
    void showTiming() const;
    void displayInitializationResult() const;

    void processObjectDetectionResult(CoordinatesMessasgPointer coordinates_ptr);
    void processNavigationResult(StringMessagePointer message_ptr);
    void processObjectGrabResult(StringMessagePointer message_ptr);
    void processObjectMovingTaskRequest(ObjectMovingTaskMessagePointer message_ptr);
    void processStateControlCommand(StringMessagePointer message_ptr);

    void controlRobotVelocity(Velocity target);
    void changeRobotBehavior(Behavior target);
    void navigateToWaypoint(String target);
    void controlRobotManipulator(ManipulatorControl const& target);
    void grabObject(Coordinate coordinate);

    static void displayDetectedObjects(CoordinatesMessasgPointer coordinates_ptr);
    static VelocityMessage createVelocityMessage(Velocity velocity);
    static StringMessage createWaypointMessage(String name);
    static StringMessage createBehaviorMessage(Behavior behavior);
    static CoordinateMessage createCoordinateMessage(Coordinate coordinate);
    static String behaviorToString(Behavior behavior);
    static String taskStateToString(TaskState taskState);
};
}  // namespace xiaohu_robot
#endif