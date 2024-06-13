#pragma once

#ifndef XIAOHU_MOVE_OBJECT_NODE_HPP
#define XIAOHU_MOVE_OBJECT_NODE_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <atomic>
#include <deque>
#include <memory>
#include <string>
#include <thread>

namespace xiaohu_robot {
inline namespace Nodes {
class ServiceModeControllerNode final: public Runnable {
public:
    struct Configs final {
        std::string amclInitPositionRequestTopic{CommonConfigs::AmclInitPositionRequestTopic};
        std::string clearCostmapsTopic{CommonConfigs::ClearCostmapsTopic};
        std::string currentTaskStateRequestTopic{CommonConfigs::CurrentTaskStateRequestTopic};
        std::string currentTaskStateResultTopic{CommonConfigs::CurrentTaskStateResultTopic};
        std::string inspectionTaskRequestTopic{CommonConfigs::InspectionTaskRequestTopic};
        std::string inspectionTaskResultTopic{CommonConfigs::InspectionTaskResultTopic};
        std::string medicineDeliveryTaskRequestTopic{CommonConfigs::MedicineDeliveryTaskRequestTopic};
        std::string medicineDeliveryTaskResultTopic{CommonConfigs::MedicineDeliveryTaskResultTopic};
        std::string initPositionRequestTopic{CommonConfigs::InitPositionRequestTopic};
        std::string initPositionResultTopic{CommonConfigs::InitPositionResultTopic};
        std::string coordinateNavigationTopic{CommonConfigs::CoordinateNavigationTopic};
        std::string velocityControlRequestTopic{CommonConfigs::VelocityControlRequestTopic};
        std::string manipulatorControlRequestTopic{CommonConfigs::ManipulatorControlRequestTopic};
        std::string medcineDetectionRequestTopic{CommonConfigs::MedicineDetectionRequestTopic};
        std::string medicineDetectionResultTopic{CommonConfigs::MedicineDetectionResultTopic};
        std::string textToSpeechRequestTopic{CommonConfigs::TextToSpeechRequestTopic};
        std::string textToSpeechResultTopic{CommonConfigs::TextToSpeechResultTopic};
        std::string medicineGraspRequestTopic{CommonConfigs::MedicineGraspRequestTopic};
        std::string medicineGraspResultTopic{CommonConfigs::MedicineGraspResultTopic};
        std::string speechRecognitionRequestTopic{CommonConfigs::SpeechRecognitionRequestTopic};
        std::string speechRecognitionResultTopic{CommonConfigs::SpeechRecognitionResultTopic};
        std::string voiceRecognitionRequestTopic{CommonConfigs::SpeechRecognitionRequestTopic};
        std::string temperatureMeasurementRequestTopic{CommonConfigs::TemperatureMeasurementRequestTopic};
        std::string temperatureMeasurementResultTopic{CommonConfigs::TemperatureMeasurementResultTopic};
        std::string videoCallingRequestTopic{CommonConfigs::VideoCallRequestTopic};
        std::string videoCallingResultTopic{CommonConfigs::VideoCallResultTopic};
        NodeBasicConfigs nodeBasicConfig{};
    };

    ServiceModeControllerNode(Configs);
    ~ServiceModeControllerNode();
    void run() override;

private:
    enum class TaskState {
        WaitingForPositionInitialization,
        ReadyToPerformTasks,
        GoingToPharmacy,
        DetectingMedicine,
        MedicineDetectionFailed,
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
        VideoCallFailed
    };

    enum class ObjectDetectionControl {
        Start,
        Stop
    };

    struct TaskStateContext final {
        std::deque<std::unique_ptr<TaskResult>> taskResults{};
        TaskState currentTaskState{TaskState::WaitingForPositionInitialization};
        TaskState previousTaskState{TaskState::WaitingForPositionInitialization};
        bool stateTransferring{false};

        TaskResult& taskResult();
        InspectionTask::Result& inspectionTaskResult();
        MedicineDeliveryTask::Result& medicineDeliveryTaskResult();
    };

    struct DelegationState {
        std::atomic_bool hasStarted{false};
        std::atomic_bool hasFailed{false};
        std::atomic_bool hasEnded{false};

        virtual ~DelegationState() = default;
        virtual void reset();

        void start();
        void end();
        void fail();
    };

    struct SpeechRecognitionContext final: public DelegationState {
        std::atomic_bool foundYes{false};
        std::atomic_bool foundNo{false};

        void reset() override;
    };

    struct MedicineDetectionAndGraspContext final: public DelegationState {
        Coordinate medicinePosition{};

        void reset() override;
    };

    struct TemperatureMeasurementContext final: public DelegationState {
        bool isRemeasuring{false};
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
    Publisher const initPositionResultPublisher;
    Publisher const amclInitPositionRequestTopicPublisher;

    TaskStateContext taskState;
    std::deque<std::unique_ptr<Task>> tasks;
    Subscriber const currentTaskStateRequestSubscriber;
    Publisher const currentTaskStateResultPublisher;
    Subscriber const inspectionTaskRequestSubscriber;
    Publisher const inspectionTaskResultPublisher;
    Subscriber const medicineDeliveryRequestSubsriber;
    Publisher const medicineDeliveryTaskResultPublisher;

    DelegationState velocityControl;
    Publisher const velocityControlRequestPublisher;

    DelegationState manipulatorControl;
    Publisher const manipulatiorControlRequestPublisher;

    std::thread navigationWaitingThread;
    ServiceClient clearCostmapsClient;
    DelegationState obstacleClearing;
    DelegationState navigation;
    NavigationClient navigationClient;
    int navigationFailedTimes;

    DelegationState textToSpeech;
    Publisher const textToSpeechRequestPublisher;
    Subscriber const textToSpeechResultSubscriber;

    SpeechRecognitionContext speechReognition;
    SpeechRecognitionContext confirmSpeechRecognition;
    Publisher const speechRecognitionRequestPublisher;
    Subscriber const speechRecognitionResultSubscriber;
    int speechRecognitionFailedTimes;

    MedicineDetectionAndGraspContext medicineDetection;
    DelegationState waitingForMedicinePreparation;
    int medicineDetectionFailedTimes;
    Publisher const medicineDetectionRequestPublisher;
    Subscriber const medicineDetectionResultSubscriber;

    MedicineDetectionAndGraspContext medicineGrasp;
    Publisher const medicineGraspRequestPublisher;
    Subscriber const medicineGraspResultSubscriber;

    TemperatureMeasurementContext temperatureMeasurement;
    Publisher const temperatureMeasurementRequestPublisher;
    Subscriber const temperatureMeasurementResultSubscriber;

    DelegationState videoCall;
    Publisher const videoCallRequestPublisher;
    Subscriber const videoCallResultSubscriber;

    DelegationState waiting;
    DelegationState exceptionHandling;

    Configs configs;

    Task& getCurrentTask() const;
    InspectionTask& getCurrentInspectionTask() const;
    MedicineDeliveryTask& getCurrentMedicineDeliveryTask() const;
    TaskState getCurrentTaskState() const;
    TaskState getPreviousTaskState() const;
    void transferCurrentTaskStateTo(TaskState nextState);
    void setPreviousTaskState(TaskState previousState);
    void checkNavigationState();
    void clearCostmaps();

    void waitForPositionInitialisation();
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
    void medicineDetectionFailed();
    void videoCallFailed();

    void showRemainedTasksCount() const;
    void showTiming() const;
    void displayInitializationResult() const;

    void whenReceivedCurrentTaskStateRequest(EmptyMessage::ConstPtr const&);
    void whenReceivedInitPositionRequest(CoordinateMessage::ConstPtr const&);
    void whenReceivedInspectionTaskRequest(InspectionTaskRequestMessage::ConstPtr const&);
    void whenReceivedMedicineDeliveryTaskRequest(MedicineDeliveryTaskRequestMessage::ConstPtr const&);
    void whenReceivedMedicineDetectionResult(ObjectDetectionResultMessasge::ConstPtr const&);
    void whenReceivedMedicineGraspResult(StringMessage::ConstPtr const&);
    void whenReceivedTextToSpeechResult(StatusAndDescriptionMessage::ConstPtr const&);
    void whenReceivedSpeechRecognitionResult(StatusAndDescriptionMessage::ConstPtr const&);
    void whenReceivedTemperatureMeasurementResult(Float64Message::ConstPtr const&);
    void whenReceivedVideoCallResult(StatusAndDescriptionMessage::ConstPtr const&);

    void delegateVelocityControl(LinearSpeed forward);
    void delegateObjectDetectionControl(ObjectDetectionControl target);
    void delegateNavigation(NavigationGoal);
    void delegateControlingRobotManipulator(ManipulatorControl const& target);
    void delegateObjectGrasping(Coordinate coordinate);
    void delegateTextToSpeech(std::string content);
    void delegateSpeechRecognition(Duration duration);
    void delegateTemperatureMeasurement();
    void delegateVideoCall();

    static void displayDetectedObjects(ObjectDetectionResultMessasge::ConstPtr const& coordinates_ptr);
    static StringMessage createWaypointMessage(std::string name);
    static StringMessage createObjectDetectionControlMessage(ObjectDetectionControl behavior);
    static std::string toString(ObjectDetectionControl behavior);
    static std::string toString(TaskState taskState);
};
}  // namespace Nodes
}  // namespace xiaohu_robot
#endif