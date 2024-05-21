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
    struct Configs final {
        std::string baseStationName{CommonConfigs::baseStationName};
        std::string legacyGeneralTasksRequestTopic{CommonConfigs::legacyGeneralTasksRequestTopic};
        std::string mappingTaskRequestTopic{CommonConfigs::mappingTaskRequestTopic};
        std::string inspectionTaskRequestTopic{CommonConfigs::inspectionTaskRequestTopic};
        std::string medicineDeliveryTaskRequestTopic{CommonConfigs::medicineDeliveryTaskRequestTopic};
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
        MeasuringTemperature,
        HasMeasuredTemperature,
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
        TaskState currentTaskState{TaskState::CalibratingInitialPosition};
        TaskState previousTaskState{TaskState::CalibratingInitialPosition};
        bool stateTransferring{false};
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
    std::deque<bool> currentTaskLegacy;
    std::deque<LegacyGeneralTask> legacyGeneralTasks;
    Subscriber const legacyGeneralTaskRequestSubscriber;
    std::deque<std::unique_ptr<SpecificTask>> tasks;
    Subscriber const inspectionTaskRequestSubscriber;
    Subscriber const mappingTaskRequestSubscriber;
    Subscriber const medicineDeliveryRequestSubsriber;

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
    Publisher const speechRecognitionRequestPublisher;
    Subscriber const speechRecognitionResultSubscriber;
    int speechRecognitionContinuousFailureTimes;

    MedicineDetectionAndGraspContext medicineDetection;
    Publisher const medicineDetectionRequestPublisher;
    Subscriber const medicineDetectionResultSubscriber;

    MedicineDetectionAndGraspContext medicineGrasp;
    Publisher const medicineGraspRequestPublisher;
    Subscriber const medicineGraspResultSubscriber;
    
    SoundplayClient soundPlayClient;

    DelegationState waiting;
    DelegationState exceptionHandling;
    DelegationState temperatureMeasurement;

    Configs configs;

    bool isCurrentTaskLegacy() const;
    SpecificTask& getCurrentTask();
    LegacyGeneralTask& getCurrentLegacyTask();
    MappingTask& getCurrentMappingTask();
    InspectionTask& getCurrentInspectionTask();
    MedicineDeliveryTask& getCurrentMedicineDeliveryTask();
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
    void measuringTemperature();
    void hasMeasuredTemperature();
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

    void whenReceivedInitPositionRequest(CoordinateMessagePointer);
    void whenReceivedInitPositionResult(StatusAndDescriptionMessagePointer);
    void whenReceivedMappingTaskRequest(MappingTaskMessagePointer);
    void whenReceivedInspectionTaskRequest(InspectionTaskMessagePointer);
    void whenReceivedMedicineDeliveryTaskRequest(MedicineDeliveryTaskMessagePointer);
    void whenReceivedMedicineDetectionResult(ObjectDetectionResultMessasgePointer);
    void whenReceivedNavigationResult(StringMessagePointer);
    void whenReceivedMedicineGraspResult(StringMessagePointer);
    void whenReceivedLegacyGeneralTaskRequest(GeneralTaskMessagePointer);
    void whenReceivedStateControlCommand(StringMessagePointer);
    void whenReceivedTextToSpeechResult(StatusAndDescriptionMessagePointer);
    void whenReceivedSpeechRecognitionResult(StatusAndDescriptionMessagePointer);

    void delegateVelocityControl(LinearSpeed forward);
    void delegateObjectDetectionControl(ObjectDetectionControl target);
    void delegateNavigatingToWaypoint(std::string target);
    void delegateControlingRobotManipulator(ManipulatorControl const& target);
    void delegateObjectGrasping(Coordinate coordinate);
    void delegateTextToSpeech(std::string content);
    void delegateSpeechRecognition(Duration duration);

    static void displayDetectedObjects(ObjectDetectionResultMessasgePointer coordinates_ptr);
    static StringMessage createWaypointMessage(std::string name);
    static StringMessage createObjectDetectionControlMessage(ObjectDetectionControl behavior);
    static std::string toString(ObjectDetectionControl behavior);
    static std::string toString(TaskState taskState);
};
}  // namespace Nodes
}  // namespace xiaohu_robot
#endif