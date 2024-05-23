#pragma once

#include <boost/any.hpp>
#include <memory>
#ifndef XIAOHU_MOVE_OBJECT_NODE_HPP
#define XIAOHU_MOVE_OBJECT_NODE_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <deque>
#include <string>

namespace xiaohu_robot {
inline namespace Nodes {
class TaskControllerNode final: public Runnable {
public:
    struct Configs final {
        std::string baseStationName{CommonConfigs::baseStationName};
        std::string currentTaskStateRequestTopic{CommonConfigs::currentTaskStateRequestTopic};
        std::string currentTaskStateResultTopic{CommonConfigs::currentTaskStateResultTopic};
        std::string legacyGeneralTasksRequestTopic{CommonConfigs::legacyGeneralTasksRequestTopic};
        std::string legacyGeneralTasksResultTopic{CommonConfigs::legacyGeneralTasksResultTopic};
        std::string mappingTaskRequestTopic{CommonConfigs::mappingTaskRequestTopic};
        std::string mappingTaskResultTopic{CommonConfigs::mappingTaskResultTopic};
        std::string inspectionTaskRequestTopic{CommonConfigs::inspectionTaskRequestTopic};
        std::string inspectionTaskResultTopic{CommonConfigs::inspectionTaskResultTopic};
        std::string medicineDeliveryTaskRequestTopic{CommonConfigs::medicineDeliveryTaskRequestTopic};
        std::string medicineDeliveryTaskResultTopic{CommonConfigs::medicineDeliveryTaskResultTopic};
        std::string initPositionRequestTopic{CommonConfigs::initPositionRequestTopic};
        std::string initPositionResultTopic{CommonConfigs::initPositionResultTopic};
        std::string coordinateNavigationTopic{CommonConfigs::coordinateNavigationTopic};
        std::string velocityControlRequestTopic{CommonConfigs::velocityControlRequestTopic};
        std::string manipulatorControlRequestTopic{CommonConfigs::manipulatorControlRequestTopic};
        std::string medcineDetectionRequestTopic{CommonConfigs::medcineDetectionRequestTopic};
        std::string medicineDetectionResultTopic{CommonConfigs::medicineDetectionResultTopic};
        std::string waypointNavigationRequestTopic{CommonConfigs::waypointNavigationRequestTopic};
        std::string waypointNavigationResultTopic{CommonConfigs::waypointNavigationResultTopic};
        std::string textToSpeechRequestTopic{CommonConfigs::textToSpeechRequestTopic};
        std::string textToSpeechResultTopic{CommonConfigs::textToSpeechResultTopic};
        std::string medicineGraspRequestTopic{CommonConfigs::medicineGraspRequestTopic};
        std::string medicineGraspResultTopic{CommonConfigs::medicineGraspResultTopic};
        std::string speechRecognitionRequestTopic{CommonConfigs::speechRecognitionRequestTopic};
        std::string speechRecognitionResultTopic{CommonConfigs::speechRecognitionResultTopic};
        std::string voiceRecognitionRequestTopic{CommonConfigs::speechRecognitionRequestTopic};
        std::string temperatureMeasurementRequestTopic{CommonConfigs::temperatureMeasurementRequestTopic};
        std::string temperatureMeasurementResultTopic{CommonConfigs::temperatureMeasurementResultTopic};
        NodeBasicConfigs nodeBasicConfig{};
    };

    TaskControllerNode(Configs);
    ~TaskControllerNode();
    void run() override;

private:
    enum class TaskState {
        CalibratingInitialPosition,
        ReadyToPerformTasks,
        GoingToPharmacy,
        DetectingMedicine,
        GraspingMedicine,
        GoingToPatient,
        AskingMeasuringTemperature,
        MeasuringTemperature,
        AskingIfVideoNeeded,
        ConfirmPatientRequest,
        VideoCommunicating,
        SpeakingPrescription,
        AskingIfGrabbedMedicine,
        DroppingMedicine,
        SteppingBackward,
        WaitingForGrabbingMedicine,
        RetractingTheArm,
        HaveFinishedPreviousTask,
        GoingToBaseStation,
        SpeechRecognitionFailed,
        WaypointUnreachable,
        GiveUpCurrentTask,
    };

    enum class ObjectDetectionControl {
        Start,
        Stop
    };

    struct TaskStateContext final {
        std::unique_ptr<TaskResult> taskResultPointer{nullptr};
        TaskState currentTaskState{TaskState::CalibratingInitialPosition};
        TaskState previousTaskState{TaskState::CalibratingInitialPosition};
        bool stateTransferring{false};

        TaskResult& taskResult();
        MappingTask::Result& mappingTaskResult();
        InspectionTask::Result& inspectionTaskResult();
        MedicineDeliveryTask::Result& medicineDeliveryTaskResult();
        LegacyGeneralTask::Result& legacyGeneralTaskResult();
    };

    struct DelegationState {
        bool hasStarted{false};
        bool hasFailed{false};
        bool hasEnded{false};

        virtual ~DelegationState() = default;
        virtual void reset();

        void start();
        void end();
        void fail();
    };

    struct SpeechRecognitionContext final: public DelegationState {
        bool foundYes{false};
        bool foundNo{false};

        void reset() override;
    };

    struct MedicineDetectionAndGraspContext final: public DelegationState {
        Coordinate medicinePosition{};

        void reset() override;
    };

    struct TemperatureMeasurementContext final: public DelegationState {
        Temperature temperature{0, UnitTemperature::celcius};

        void reset() override;
    };

    struct UnsupportedTaskException final: public std::runtime_error {
        UnsupportedTaskException():
            std::runtime_error("未支持的任务类型") {}
    };

    NodeHandle nodeHandle;
    NodeTiming nodeTiming;

    DelegationState initPosition;
    Coordinate baseStatePosition;
    Subscriber const initPositionRequestSubscriber;
    Subscriber const initPositionResultSubscriber;

    TaskStateContext taskState;
    std::deque<std::unique_ptr<Task>> tasks;
    Subscriber const currentTaskStateRequestSubscriber;
    Publisher const currentTaskStateResultPublisher;
    Subscriber const legacyGeneralTaskRequestSubscriber;
    Publisher const legacyGeneralTaskResultPublisher;
    Subscriber const inspectionTaskRequestSubscriber;
    Publisher const inspectionTaskResultPublisher;
    Subscriber const mappingTaskRequestSubscriber;
    Publisher const mappingTaskResultPublisher;
    Subscriber const medicineDeliveryRequestSubsriber;
    Publisher const medicineDeliveryTaskResultPublisher;

    DelegationState velocityControl;
    Publisher const velocityControlRequestPublisher;

    DelegationState manipulatorControl;
    Publisher const manipulatiorControlRequestPublisher;

    DelegationState waypointNavigation;
    Publisher const waypointNavigationRequestPublisher;
    Subscriber const waypointNavigationResultSubscriber;
    int waypointUnreachableTimes;

    DelegationState obstacleClearing;
    DelegationState navigation;
    NavigationClient navigationClient;

    DelegationState textToSpeech;
    Publisher const textToSpeechRequestPublisher;
    Subscriber const textToSpeechResultSubscriber;

    SpeechRecognitionContext speechReognition;
    SpeechRecognitionContext confirmSpeechRecognition;
    Publisher const speechRecognitionRequestPublisher;
    Subscriber const speechRecognitionResultSubscriber;
    int speechRecognitionContinuousFailureTimes;

    MedicineDetectionAndGraspContext medicineDetection;
    Publisher const medicineDetectionRequestPublisher;
    Subscriber const medicineDetectionResultSubscriber;

    MedicineDetectionAndGraspContext medicineGrasp;
    Publisher const medicineGraspRequestPublisher;
    Subscriber const medicineGraspResultSubscriber;

    TemperatureMeasurementContext temperatureMeasurement;
    Publisher const temperatureMeasurementRequestPublisher;
    Subscriber const temperatureMeasurementResultSubscriber;

    DelegationState waiting;
    DelegationState exceptionHandling;

    Configs configs;

    Task& getCurrentTask() const;
    bool isCurrentTaskLegacy() const;
    LegacyGeneralTask& getCurrentLegacyTask() const;
    MappingTask& getCurrentMappingTask() const;
    InspectionTask& getCurrentInspectionTask() const;
    MedicineDeliveryTask& getCurrentMedicineDeliveryTask() const;
    TaskState getCurrentTaskState() const;
    TaskState getPreviousTaskState() const;
    void transferCurrentTaskStateTo(TaskState nextState);
    void setPreviousTaskState(TaskState previousState);
    void checkNavigationState();

    void startInitialPositionCalibration();
    void readyToPerformTasks();
    void goToPharmacy();
    void detectMedicine();
    void graspMedicine();
    void goToPatient();
    void dropMedicine();
    void stepBackward();
    void retractManipulaor();
    void haveFinishedPreviousTask();
    void goToBaseStation();
    void waypointUnreachable();
    void askMeasuringTemperature();
    void measureTemperature();
    void askIfVideoNeeded();
    void confirmPatientRequest();
    void videoCommunicating();
    void speechRecognitionFailed();
    void giveUpCurrentTask();
    void speakPrescription();
    void askIfGrabbedMedicine();
    void waitForGrabbingMedicine();

    void showRemainedTasksCount() const;
    void showTiming() const;
    void displayInitializationResult() const;

    void whenReceivedCurrentTaskStateRequest(EmptyMessage::ConstPtr const&);
    void whenReceivedInitPositionRequest(CoordinateMessage::ConstPtr const&);
    void whenReceivedInitPositionResult(StatusAndDescriptionMessage::ConstPtr const&);
    void whenReceivedMappingTaskRequest(MappingTaskRequestMessage::ConstPtr const&);
    void whenReceivedInspectionTaskRequest(InspectionTaskRequestMessage::ConstPtr const&);
    void whenReceivedMedicineDeliveryTaskRequest(MedicineDeliveryTaskRequestMessage::ConstPtr const&);
    void whenReceivedMedicineDetectionResult(ObjectDetectionResultMessasge::ConstPtr const&);
    void whenReceivedNavigationResult(StringMessage::ConstPtr const&);
    void whenReceivedMedicineGraspResult(StringMessage::ConstPtr const&);
    void whenReceivedLegacyGeneralTaskRequest(LegacyGeneralTaskRequestMessage::ConstPtr const&);
    void whenReceivedTextToSpeechResult(StatusAndDescriptionMessage::ConstPtr const&);
    void whenReceivedSpeechRecognitionResult(StatusAndDescriptionMessage::ConstPtr const&);
    void whenReceivedTemperatureMeasurementResult(Float64Message::ConstPtr const&);

    void delegateVelocityControl(LinearSpeed forward);
    void delegateObjectDetectionControl(ObjectDetectionControl target);
    void delegateNavigatingToWaypoint(std::string target);
    void delegateControlingRobotManipulator(ManipulatorControl const& target);
    void delegateObjectGrasping(Coordinate coordinate);
    void delegateTextToSpeech(std::string content);
    void delegateSpeechRecognition(Duration duration);
    void delegateTemperatureMeasurement();

    static void displayDetectedObjects(ObjectDetectionResultMessasge::ConstPtr const& coordinates_ptr);
    static StringMessage createWaypointMessage(std::string name);
    static StringMessage createObjectDetectionControlMessage(ObjectDetectionControl behavior);
    static std::string toString(ObjectDetectionControl behavior);
    static std::string toString(TaskState taskState);
};
}  // namespace Nodes
}  // namespace xiaohu_robot
#endif