#include "xiaohu_robot/Nodes/ServiceModeControllerNode.hpp"
#include "ros/console.h"
#include "ros/duration.h"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Exceptions.hpp"
#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/VelocityCommand.hpp"
#include <clocale>
#include <cstdlib>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");
    ros::init(argc, argv, CommonConfigs::ServiceModeControllerNodeName);
    ServiceModeControllerNode taskControllerNode{{}};
    taskControllerNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
ServiceModeControllerNode::ServiceModeControllerNode(Configs configs):
    nodeHandle(configs.nodeBasicConfig.nodeNamespace),
    nodeTiming{configs.nodeBasicConfig.loopFrequency},
    enableServiceModeResultPublisher(nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.enableServiceModeResultTopic,
        configs.nodeBasicConfig.messageBufferSize
    )),
    initPosition{},
    baseStatePosition{},
    initPositionRequestSubscriber{nodeHandle.subscribe<CoordinateMessage>(
        configs.initPositionRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedInitPositionRequest,
        this
    )},
    initPositionResultPublisher{nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.initPositionResultTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    amclInitPositionRequestTopicPublisher{nodeHandle.advertise<CoordinateCovarianceStampedMessage>(
        configs.amclInitPositionRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    taskState{},
    tasks{},
    currentTaskStateRequestSubscriber{nodeHandle.subscribe<EmptyMessage>(
        configs.currentTaskStateRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedCurrentTaskStateRequest,
        this
    )},
    currentTaskStateResultPublisher{nodeHandle.advertise<StringMessage>(
        configs.currentTaskStateResultTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    inspectionTaskRequestSubscriber{nodeHandle.subscribe<InspectionTaskRequestMessage>(
        configs.inspectionTaskRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedInspectionTaskRequest,
        this
    )},
    inspectionTaskResultPublisher{nodeHandle.advertise<InspectionTaskResultMessage>(
        configs.inspectionTaskResultTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    medicineDeliveryRequestSubsriber{nodeHandle.subscribe<MedicineDeliveryTaskRequestMessage>(
        configs.medicineDeliveryTaskRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedMedicineDeliveryTaskRequest,
        this
    )},
    medicineDeliveryTaskResultPublisher{nodeHandle.advertise<MedicineDeliveryTaskResultMessage>(
        configs.medicineDeliveryTaskResultTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    velocityControl{},
    velocityControlRequestPublisher{nodeHandle.advertise<VelocityCommandMessage>(
        configs.velocityControlRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    manipulatorControl{},
    manipulatiorControlRequestPublisher{nodeHandle.advertise<ManipulatorControlMessage>(
        configs.manipulatorControlRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    clearCostmapsClient{nodeHandle.serviceClient<std_srvs::Empty>(configs.clearCostmapsTopic)},
    navigationFailedTimes{0},
    obstacleClearing{},
    navigation{},
    navigationClient{configs.coordinateNavigationTopic},
    textToSpeech{},
    textToSpeechRequestPublisher{
        nodeHandle.advertise<StringMessage>(configs.textToSpeechRequestTopic, configs.nodeBasicConfig.messageBufferSize)
    },
    textToSpeechResultSubscriber{nodeHandle.subscribe<StatusAndDescriptionMessage>(
        configs.textToSpeechResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedTextToSpeechResult,
        this
    )},
    speechReognition{},
    speechRecognitionRequestPublisher{nodeHandle.advertise<StringMessage>(
        configs.speechRecognitionRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    speechRecognitionResultSubscriber{nodeHandle.subscribe<StatusAndDescriptionMessage>(
        configs.speechRecognitionResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedSpeechRecognitionResult,
        this
    )},
    speechRecognitionFailedTimes{0},
    medicineDetection{},
    waitingForMedicinePreparation{},
    medicineDetectionFailedTimes{0},
    medicineDetectionRequestPublisher{nodeHandle.advertise<StringMessage>(
        configs.medcineDetectionRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    medicineDetectionResultSubscriber{nodeHandle.subscribe<ObjectDetectionResultMessasge::ConstPtr const&>(
        configs.medicineDetectionResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedMedicineDetectionResult,
        this
    )},
    medicineGrasp{},
    medicineGraspRequestPublisher{nodeHandle.advertise<CoordinateMessage>(
        configs.medicineGraspRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    medicineGraspResultSubscriber{nodeHandle.subscribe<StringMessage::ConstPtr const&>(
        configs.medicineGraspResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedMedicineGraspResult,
        this
    )},
    temperatureMeasurement{},
    temperatureMeasurementRequestPublisher{nodeHandle.advertise<EmptyMessage>(
        configs.temperatureMeasurementRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    temperatureMeasurementResultSubscriber{nodeHandle.subscribe<Float64Message>(
        configs.temperatureMeasurementResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedTemperatureMeasurementResult,
        this
    )},
    videoCall{},
    videoCallRequestPublisher{
        nodeHandle.advertise<EmptyMessage>(configs.videoCallingRequestTopic, configs.nodeBasicConfig.messageBufferSize)
    },
    videoCallResultSubscriber{nodeHandle.subscribe<StatusAndDescriptionMessage>(
        configs.videoCallingResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &ServiceModeControllerNode::whenReceivedVideoCallResult,
        this
    )},
    waiting{},
    exceptionHandling{},
    configs{std::move(configs)} {
    std::this_thread::sleep_for(std::chrono::seconds(1));
    StatusAndDescriptionMessage result;
    result.status = StatusAndDescriptionMessage::done;
    enableServiceModeResultPublisher.publish(result);
    std::cout << "任务控制器已启动。" << std::endl;
}

ServiceModeControllerNode::~ServiceModeControllerNode() {
    std::cout << "任务控制器已退出。" << std::endl;
}

TaskResult& ServiceModeControllerNode::TaskStateContext::taskResult() {
    return dynamic_cast<TaskResult&>(*taskResults.front());
}

InspectionTask::Result& ServiceModeControllerNode::TaskStateContext::inspectionTaskResult() {
    return dynamic_cast<InspectionTask::Result&>(taskResult());
}

MedicineDeliveryTask::Result& ServiceModeControllerNode::TaskStateContext::medicineDeliveryTaskResult() {
    return dynamic_cast<MedicineDeliveryTask::Result&>(taskResult());
}

void ServiceModeControllerNode::DelegationState::reset() {
    hasStarted = false;
    hasFailed = false;
    hasEnded = false;
};

void ServiceModeControllerNode::DelegationState::start() {
    hasStarted = true;
}

void ServiceModeControllerNode::DelegationState::end() {
    hasEnded = true;
}

void ServiceModeControllerNode::DelegationState::fail() {
    hasFailed = true;
}

void ServiceModeControllerNode::SpeechRecognitionContext::reset() {
    DelegationState::reset();
    foundYes = false;
    foundNo = false;
}

void ServiceModeControllerNode::MedicineDetectionAndGraspContext::reset() {
    DelegationState::reset();
    medicinePosition = Coordinate{};
}

void ServiceModeControllerNode::TemperatureMeasurementContext::reset() {
    DelegationState::reset();
    isRemeasuring = false;
    temperature = Temperature{0, UnitTemperature::celcius};
}

void ServiceModeControllerNode::run() {
    ros::Rate loopRate{configs.nodeBasicConfig.loopFrequency};
    while (ros::ok()) {
        switch (getCurrentTaskState()) {
        case TaskState::WaitingForPositionInitialization:
            waitForPositionInitialisation();
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
        case TaskState::GraspingMedicine:
            graspMedicine();
            break;
        case TaskState::GoingToPatient:
            goToPatient();
            break;
        case TaskState::DroppingMedicine:
            dropMedicine();
            break;
        case TaskState::SteppingBackward:
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
        case TaskState::AskingMeasuringTemperature:
            askMeasuringTemperature();
            break;
        case TaskState::AskingIfVideoNeeded:
            askIfVideoNeeded();
            break;
        case TaskState::VideoCommunicating:
            videoCommunicating();
            break;
        case TaskState::MeasuringTemperature:
            measureTemperature();
            break;
        case TaskState::ConfirmPatientRequest:
            confirmPatientRequest();
            break;
        case TaskState::SpeechRecognitionFailed:
            speechRecognitionFailed();
            break;
        case TaskState::GiveUpCurrentTask:
            giveUpCurrentTask();
            break;
        case TaskState::SpeakingPrescription:
            speakPrescription();
            break;
        case TaskState::AskingIfGrabbedMedicine:
            askIfGrabbedMedicine();
            break;
        case TaskState::WaitingForGrabbingMedicine:
            waitForGrabbingMedicine();
            break;
        case TaskState::MedicineDetectionFailed:
            medicineDetectionFailed();
            break;
        case TaskState::VideoCallFailed:
            videoCallFailed();
            break;
        }
        // checkNavigationState();
        ros::spinOnce();
        if (!taskState.stateTransferring) {
            nodeTiming.increment();
        } else {
            taskState.stateTransferring = false;
        }
        loopRate.sleep();
    }
}

Task& ServiceModeControllerNode::getCurrentTask() const {
    return *tasks.front();
}

InspectionTask& ServiceModeControllerNode::getCurrentInspectionTask() const {
    return dynamic_cast<InspectionTask&>(getCurrentTask());
}

MedicineDeliveryTask& ServiceModeControllerNode::getCurrentMedicineDeliveryTask() const {
    return dynamic_cast<MedicineDeliveryTask&>(getCurrentTask());
}

ServiceModeControllerNode::TaskState ServiceModeControllerNode::getCurrentTaskState() const {
    return taskState.currentTaskState;
}

ServiceModeControllerNode::TaskState ServiceModeControllerNode::getPreviousTaskState() const {
    return taskState.previousTaskState;
}

void ServiceModeControllerNode::clearCostmaps() {
    Procedure clearCostmapsService{};
    if (!clearCostmapsClient.call(clearCostmapsService)) {
        ROS_ERROR("清理代价地图失败。");
        printMessageThenThrowRuntimeError("清理代价地图失败。");
    } else {
        ROS_INFO("清理代价地图成功。");
    }
}

void ServiceModeControllerNode::transferCurrentTaskStateTo(TaskState next) {
    taskState.previousTaskState = taskState.currentTaskState;
    taskState.currentTaskState = next;
    taskState.stateTransferring = true;

    if (!taskState.taskResults.empty()) {
        taskState.taskResult().logger << toString(taskState.currentTaskState);
    }

    StringMessage message;
    message.data = toString(taskState.currentTaskState);
    currentTaskStateResultPublisher.publish(message);

    ROS_INFO(
        "任务状态: %s -> %s",
        toString(taskState.previousTaskState).c_str(),
        toString(taskState.currentTaskState).c_str()
    );
}

void ServiceModeControllerNode::setPreviousTaskState(TaskState previous) {
    taskState.previousTaskState = previous;
}

void ServiceModeControllerNode::checkNavigationState() {
    if (!navigation.hasStarted) {
        return;
    }
    GoalState goalState{navigationClient.getState()};
    if (!goalState.isDone()) {
        return;
    } else if (goalState == GoalState::SUCCEEDED) {
        navigation.end();
    } else {
        navigation.fail();
    }
}

void ServiceModeControllerNode::waitForPositionInitialisation() {
    if (!initPosition.hasStarted) {
        initPosition.start();
        ROS_INFO("服务模式已启动。");
    } else if (initPosition.hasEnded) {
        transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
        initPosition.reset();
    }
}

void ServiceModeControllerNode::readyToPerformTasks() {
    if (tasks.empty()) {
        return;
    }
    switch (getCurrentTask().getTaskType()) {
    case TaskType::Inspection:
        transferCurrentTaskStateTo(TaskState::GoingToPatient);
        break;
    case TaskType::MedicineDelivery:
        transferCurrentTaskStateTo(TaskState::GoingToPharmacy);
        break;
    }
}

void ServiceModeControllerNode::goToPharmacy() {
    if (!navigation.hasStarted) {
        delegateNavigation(getCurrentMedicineDeliveryTask().pharmacyPosition.toNavigationGoal());
    } else if (navigation.hasEnded) {
        transferCurrentTaskStateTo(TaskState::DetectingMedicine);
        navigationWaitingThread.join();
        navigation.reset();
    } else if (navigation.hasFailed) {
        transferCurrentTaskStateTo(TaskState::WaypointUnreachable);
        navigationWaitingThread.join();
        navigation.reset();
    }
}

void ServiceModeControllerNode::detectMedicine() {
    if (!medicineDetection.hasStarted) {
        delegateObjectDetectionControl(ObjectDetectionControl::Start);
    } else if (medicineDetection.hasEnded) {
        medicineGrasp.medicinePosition = medicineDetection.medicinePosition;
        delegateObjectDetectionControl(ObjectDetectionControl::Stop);
        transferCurrentTaskStateTo(TaskState::GraspingMedicine);
        medicineDetection.reset();
    } else if (medicineDetection.hasFailed) {
        transferCurrentTaskStateTo(TaskState::MedicineDetectionFailed);
        medicineDetection.reset();
    }
}

void ServiceModeControllerNode::graspMedicine() {
    if (!medicineGrasp.hasStarted) {
        delegateObjectGrasping(medicineGrasp.medicinePosition);
    } else if (medicineGrasp.hasEnded) {
        transferCurrentTaskStateTo(TaskState::GoingToPatient);
        taskState.medicineDeliveryTaskResult().fetchedMedicine = true;
        medicineGrasp.reset();
    }
}

void ServiceModeControllerNode::goToPatient() {
    if (!navigation.hasStarted) {
        switch (getCurrentTask().getTaskType()) {
        case TaskType::Inspection:
            delegateNavigation(getCurrentInspectionTask().patientPosition.toNavigationGoal());
            break;
        case TaskType::MedicineDelivery:
            delegateNavigation(getCurrentMedicineDeliveryTask().patientPosition.toNavigationGoal());
            break;
        }
    } else if (navigation.hasEnded) {
        switch (getCurrentTask().getTaskType()) {
        case TaskType::Inspection:
            transferCurrentTaskStateTo(TaskState::AskingMeasuringTemperature);
            break;
        case TaskType::MedicineDelivery:
            transferCurrentTaskStateTo(TaskState::SpeakingPrescription);
            break;
        }
        navigationWaitingThread.join();
        navigation.reset();
    } else if (navigation.hasFailed) {
        transferCurrentTaskStateTo(TaskState::WaypointUnreachable);
        navigationWaitingThread.join();
        navigation.reset();
    }
}

void ServiceModeControllerNode::speakPrescription() {
    if (!textToSpeech.hasStarted) {
        std::ostringstream stringContent{};
        stringContent << "患者" << getCurrentMedicineDeliveryTask().patientName << "，您有一份药品待取：";
        stringContent << getCurrentMedicineDeliveryTask().prescription << "……";
        stringContent << "请您伸手取药。";
        delegateTextToSpeech(stringContent.str());
    } else if (textToSpeech.hasEnded) {
        transferCurrentTaskStateTo(TaskState::AskingIfGrabbedMedicine);
        textToSpeech.reset();
    }
}

void ServiceModeControllerNode::askIfGrabbedMedicine() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("您是否已准备好取药？");
    } else if (textToSpeech.hasEnded && !speechReognition.hasStarted) {
        delegateSpeechRecognition(10_s);
    } else if (textToSpeech.hasEnded && speechReognition.hasEnded) {
        if (speechReognition.foundYes) {
            transferCurrentTaskStateTo(TaskState::DroppingMedicine);
        } else {
            transferCurrentTaskStateTo(TaskState::WaitingForGrabbingMedicine);
        }
        speechRecognitionFailedTimes = 0;
        textToSpeech.reset();
        speechReognition.reset();
    } else if (textToSpeech.hasEnded && speechReognition.hasFailed) {
        transferCurrentTaskStateTo(TaskState::SpeechRecognitionFailed);
        textToSpeech.reset();
        speechReognition.reset();
    }
}

void ServiceModeControllerNode::waitForGrabbingMedicine() {
    if (!textToSpeech.hasStarted && !waiting.hasStarted) {
        delegateTextToSpeech("好的，我会再等待您十秒钟。");
        waiting.start();
        nodeTiming.addTimedTask(10_s, [this]() { waiting.end(); }, "等待患者抓取药品结束");
    } else if (textToSpeech.hasEnded && waiting.hasEnded) {
        transferCurrentTaskStateTo(TaskState::AskingIfGrabbedMedicine);
        textToSpeech.reset();
        waiting.reset();
    }
}

void ServiceModeControllerNode::dropMedicine() {
    if (!manipulatorControl.hasStarted) {
        delegateControlingRobotManipulator(GripperControl{20_cm});
        nodeTiming.addTimedTask(3_s, [this]() { manipulatorControl.end(); }, "结束张开机械手");
    } else if (manipulatorControl.hasEnded) {
        transferCurrentTaskStateTo(TaskState::SteppingBackward);
        taskState.medicineDeliveryTaskResult().deliveredMedicine = true;
        manipulatorControl.reset();
    }
}

void ServiceModeControllerNode::stepBackward() {
    if (!velocityControl.hasStarted) {
        delegateVelocityControl(-0.1_m_per_s);
        nodeTiming.addTimedTask(
            5_s,
            [this]() {
                delegateVelocityControl(0_m_per_s);
                velocityControl.end();
            },
            "停止后退"
        );
    } else if (velocityControl.hasEnded) {
        transferCurrentTaskStateTo(TaskState::RetractingTheArm);
        velocityControl.reset();
    }
}

void ServiceModeControllerNode::retractManipulaor() {
    if (!manipulatorControl.hasStarted) {
        delegateControlingRobotManipulator(
            MultiPartControl{std::make_unique<ArmControl>(0_cm), std::make_unique<GripperControl>(0_cm)}
        );
        nodeTiming.addTimedTask(5_s, [this]() { manipulatorControl.end(); }, "停止回收机械臂");
    } else if (manipulatorControl.hasEnded) {
        transferCurrentTaskStateTo(TaskState::HaveFinishedPreviousTask);
        manipulatorControl.reset();
    }
}

void ServiceModeControllerNode::haveFinishedPreviousTask() {
    ROS_INFO("已完成任务:\n%s", getCurrentTask().toString().c_str());
    switch (getCurrentTask().getTaskType()) {
    case TaskType::Inspection:
        inspectionTaskResultPublisher.publish(taskState.inspectionTaskResult().toMessage());
        break;
    case TaskType::MedicineDelivery:
        medicineDeliveryTaskResultPublisher.publish(taskState.medicineDeliveryTaskResult().toMessage());
        break;
    }
    ROS_INFO("已发布任务结果。");
    taskState.taskResults.pop_front();
    tasks.pop_front();
    showRemainedTasksCount();
    if (!tasks.empty()) {
        transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
    } else {
        transferCurrentTaskStateTo(TaskState::GoingToBaseStation);
    }
}

void ServiceModeControllerNode::askIfVideoNeeded() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("请问您是否需要与医生视频通话？");
    } else if (textToSpeech.hasEnded && !speechReognition.hasStarted) {
        delegateSpeechRecognition(10_s);
    } else if (speechReognition.hasFailed) {
        transferCurrentTaskStateTo(TaskState::SpeechRecognitionFailed);
        speechReognition.reset();
        textToSpeech.reset();
    } else if (speechReognition.hasEnded) {
        transferCurrentTaskStateTo(TaskState::ConfirmPatientRequest);
        bool foundYes{speechReognition.foundYes};
        confirmSpeechRecognition.foundYes = foundYes;
        speechRecognitionFailedTimes = 0;
        speechReognition.reset();
        textToSpeech.reset();
    }
}

void ServiceModeControllerNode::speechRecognitionFailed() {
    if (!textToSpeech.hasStarted) {
        if (speechRecognitionFailedTimes < 3) {
            delegateTextToSpeech("抱歉，我没有听清，请再试一次。您可以说“是”，或者“否”。");
        } else {
            delegateTextToSpeech("语音检测连续失败 3 次，已放弃当前任务。");
        }
    } else if (textToSpeech.hasEnded) {
        if (speechRecognitionFailedTimes < 3) {
            transferCurrentTaskStateTo(getPreviousTaskState());
            speechRecognitionFailedTimes++;
            textToSpeech.reset();
        } else {
            transferCurrentTaskStateTo(TaskState::GiveUpCurrentTask);
            speechRecognitionFailedTimes = 0;
            textToSpeech.reset();
        }
    }
}

void ServiceModeControllerNode::giveUpCurrentTask() {
    if (!textToSpeech.hasStarted && !exceptionHandling.hasStarted) {
        delegateTextToSpeech("重复出现异常。任务中断。请勿接触机器人的机械臂。");
        exceptionHandling.start();
    } else if (textToSpeech.hasEnded && exceptionHandling.hasEnded) {
        transferCurrentTaskStateTo(TaskState::RetractingTheArm);
        textToSpeech.reset();
        exceptionHandling.reset();
    }
}

void ServiceModeControllerNode::confirmPatientRequest() {
    if (!textToSpeech.hasStarted) {
        if (confirmSpeechRecognition.foundYes) {
            delegateTextToSpeech("好的，已为您安排视频通话。");
        } else {
            delegateTextToSpeech("好的，已取消视频通话。");
        }
    } else if (textToSpeech.hasEnded) {
        if (confirmSpeechRecognition.foundYes) {
            transferCurrentTaskStateTo(TaskState::VideoCommunicating);
        } else {
            taskState.inspectionTaskResult().calledDoctor = false;
            transferCurrentTaskStateTo(TaskState::HaveFinishedPreviousTask);
        }
        confirmSpeechRecognition.reset();
        textToSpeech.reset();
    }
}

void ServiceModeControllerNode::videoCommunicating() {
    if (!videoCall.hasStarted) {
        delegateVideoCall();
    } else if (videoCall.hasEnded) {
        taskState.inspectionTaskResult().calledDoctor = true;
        transferCurrentTaskStateTo(TaskState::HaveFinishedPreviousTask);
        videoCall.reset();
    } else if (videoCall.hasFailed) {
        taskState.inspectionTaskResult().calledDoctor = false;
        transferCurrentTaskStateTo(TaskState::HaveFinishedPreviousTask);
        videoCall.reset();
    }
}

void ServiceModeControllerNode::goToBaseStation() {
    if (tasks.empty()) {
        if (!navigation.hasStarted) {
            delegateNavigation(baseStatePosition.toNavigationGoal());
        } else if (navigation.hasEnded) {
            transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
            navigationWaitingThread.join();
            navigation.reset();
        } else if (navigation.hasFailed) {
            transferCurrentTaskStateTo(TaskState::WaypointUnreachable);
            navigationWaitingThread.join();
            navigation.reset();
        }
    } else {
        if (!navigation.hasStarted) {
            transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
        } else {
            if (navigation.hasEnded) {
                transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
                navigationWaitingThread.join();
                navigation.reset();
            } else {
                if (!navigation.hasFailed) {
                    navigationClient.cancelGoal();
                } else {
                    transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
                    navigationWaitingThread.join();
                    navigation.reset();
                }
            }
        }
    }
}

void ServiceModeControllerNode::waypointUnreachable() {
    if (!textToSpeech.hasStarted) {
        if (navigationFailedTimes < 3) {
            delegateTextToSpeech("无法前往下一处航点，请尝试移除周围的障碍物，机器人会在 20 秒内重试。");
        } else {
            delegateTextToSpeech("导航连续失败 3 次，已放弃本次任务。");
        }
    } else if (textToSpeech.hasEnded) {
        if (navigationFailedTimes < 3) {
            if (!obstacleClearing.hasStarted) {
                obstacleClearing.start();
                nodeTiming.addTimedTask(20_s, [this]() { obstacleClearing.end(); }, "等待清除障碍物结束");
            } else if (obstacleClearing.hasEnded) {
                transferCurrentTaskStateTo(getPreviousTaskState());
                navigationFailedTimes++;
                obstacleClearing.reset();
                textToSpeech.reset();
            }
        } else {
            transferCurrentTaskStateTo(TaskState::GiveUpCurrentTask);
            navigationFailedTimes = 0;
            textToSpeech.reset();
        }
    }
}

void ServiceModeControllerNode::askMeasuringTemperature() {
    if (!textToSpeech.hasStarted) {
        if (!temperatureMeasurement.isRemeasuring) {
            std::ostringstream stringContent{};
            stringContent << getCurrentInspectionTask().patientName << "，您好。小护正在执行巡检任务。";
            stringContent << "请将手腕对准温度传感器测量体温。";
            delegateTextToSpeech(stringContent.str());
        } else {
            delegateTextToSpeech("重新测量体温中。");
        }
    } else if (textToSpeech.hasEnded) {
        transferCurrentTaskStateTo(TaskState::MeasuringTemperature);
        textToSpeech.reset();
    }
}

void ServiceModeControllerNode::measureTemperature() {
    if (!temperatureMeasurement.hasStarted) {
        delegateTemperatureMeasurement();
    } else if (temperatureMeasurement.hasEnded && !textToSpeech.hasStarted) {
        std::ostringstream stringContent{};
        stringContent << "测量的体温为：" << std::fixed << std::setprecision(2)
                      << temperatureMeasurement.temperature.getValue() << "摄氏度。";
        if (temperatureMeasurement.temperature < Temperature{36.0, UnitTemperature::celcius}) {
            stringContent << "体温数值异常偏低，您需要将手腕紧贴传感器探头。";
            temperatureMeasurement.isRemeasuring = true;
        } else if (temperatureMeasurement.temperature < Temperature{37.3, UnitTemperature::celcius}) {
            stringContent << "体温正常。";
        } else {
            stringContent << "您发烧了。";
        }
        delegateTextToSpeech(stringContent.str());
    } else if (textToSpeech.hasEnded) {
        if (temperatureMeasurement.temperature < Temperature{34.0, UnitTemperature::celcius}) {
            transferCurrentTaskStateTo(TaskState::AskingMeasuringTemperature);
            temperatureMeasurement.reset();
            temperatureMeasurement.isRemeasuring = true;
        } else {
            transferCurrentTaskStateTo(TaskState::AskingIfVideoNeeded);
            taskState.inspectionTaskResult().measuredTemperature = true;
            taskState.inspectionTaskResult().patientTemperature = temperatureMeasurement.temperature;
            temperatureMeasurement.reset();
        }
        textToSpeech.reset();
    }
}

void ServiceModeControllerNode::medicineDetectionFailed() {
    if (!textToSpeech.hasStarted) {
        if (medicineDetectionFailedTimes < 3) {
            delegateTextToSpeech("无法检测到药品，请尝试将药品摆放至机器人前方的桌面上，机器人会在 30 秒内重试。");
        } else {
            delegateTextToSpeech("药品检测连续失败 3 次，已放弃当前任务。");
        }
    } else if (textToSpeech.hasEnded) {
        if (medicineDetectionFailedTimes < 3) {
            if (!waitingForMedicinePreparation.hasStarted) {
                waitingForMedicinePreparation.start();
                nodeTiming.addTimedTask(30_s, [this]() { waitingForMedicinePreparation.end(); }, "等待清除障碍物结束");
            } else if (waitingForMedicinePreparation.hasEnded) {
                transferCurrentTaskStateTo(getPreviousTaskState());
                medicineDetectionFailedTimes++;
                waitingForMedicinePreparation.reset();
                textToSpeech.reset();
            }
        } else {
            medicineDetectionFailedTimes = 0;
            transferCurrentTaskStateTo(TaskState::GiveUpCurrentTask);
            textToSpeech.reset();
        }
    }
}

void ServiceModeControllerNode::videoCallFailed() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("暂时无法实现视频通话，但您的需求已被记录。");
    } else if (textToSpeech.hasEnded) {
        transferCurrentTaskStateTo(TaskState::HaveFinishedPreviousTask);
        textToSpeech.reset();
    }
}

void ServiceModeControllerNode::showRemainedTasksCount() const {
    if (tasks.empty())
        ROS_INFO("没有任务可被执行。");
    else {
        ROS_INFO("剩余任务数量: %zd", tasks.size());
    }
}

void ServiceModeControllerNode::showTiming() const {
    ROS_INFO("当前状态计时: %s", nodeTiming.getCurrentTiming().toString().c_str());
}

void ServiceModeControllerNode::displayInitializationResult() const {
    ROS_INFO("任务控制器已初始化。");
}

void ServiceModeControllerNode::whenReceivedCurrentTaskStateRequest(EmptyMessage::ConstPtr const&) {
    StringMessage message;
    message.data = toString(taskState.currentTaskState);
    currentTaskStateResultPublisher.publish<StringMessage>(message);
}

void ServiceModeControllerNode::whenReceivedInitPositionRequest(CoordinateMessage::ConstPtr const& message) {
    baseStatePosition = Coordinate{message};
    CoordinateCovarianceStampedMessage amclInitPositionRequest;
    amclInitPositionRequest.header.frame_id = "map";
    amclInitPositionRequest.header.stamp = ros::Time::now();
    amclInitPositionRequest.pose.pose = *message;
    amclInitPositionRequest.pose.covariance.fill(0.0);
    amclInitPositionRequestTopicPublisher.publish(amclInitPositionRequest);
    nodeTiming.addTimedTask(1_s, [this]() { clearCostmaps(); }, "清除代价地图中");
    nodeTiming.addTimedTask(2_s, [this]() { initPosition.end(); }, "坐标初始化完成");
    ROS_INFO("充电基站坐标已初始化。");
    std::cout << baseStatePosition.toString() << std::endl;
}

void ServiceModeControllerNode::whenReceivedInspectionTaskRequest(InspectionTaskRequestMessage::ConstPtr const& message
) {
    tasks.emplace_back(std::make_unique<InspectionTask>(message));
    taskState.taskResults.push_back(std::make_unique<InspectionTask::Result>(
        dynamic_cast<InspectionTask&>(*tasks.back())
    ));
    ROS_INFO("收到一个任务:\n%s", tasks.back()->toString().c_str());
}

void ServiceModeControllerNode::whenReceivedMedicineDeliveryTaskRequest(
    MedicineDeliveryTaskRequestMessage::ConstPtr const& message
) {
    tasks.emplace_back(std::make_unique<MedicineDeliveryTask>(message));
    taskState.taskResults.push_back(std::make_unique<MedicineDeliveryTask::Result>(
        dynamic_cast<MedicineDeliveryTask&>(*tasks.back())
    ));
    ROS_INFO("收到一个任务:\n%s", tasks.back()->toString().c_str());
}

void ServiceModeControllerNode::whenReceivedMedicineDetectionResult(
    ObjectDetectionResultMessasge::ConstPtr const& coordinates
) {
    if (!medicineDetection.hasStarted) {
        return;
    }
    if (!coordinates->x.empty()) {
        displayDetectedObjects(coordinates);
        constexpr std::size_t nearest_object_index{0};
        medicineDetection.medicinePosition = Coordinate{
            coordinates->x[nearest_object_index],
            coordinates->y[nearest_object_index],
            coordinates->z[nearest_object_index]
        };
        medicineDetection.end();
    } else {
        medicineDetection.fail();
    }
}

void ServiceModeControllerNode::whenReceivedMedicineGraspResult(StringMessage::ConstPtr const& message) {
    if (!medicineGrasp.hasStarted) {
        return;
    }
    if (message->data == "done") {
        medicineGrasp.end();
    } else {
        medicineGrasp.fail();
    }
}

void ServiceModeControllerNode::whenReceivedTextToSpeechResult(StatusAndDescriptionMessage::ConstPtr const& message) {
    if (!textToSpeech.hasStarted) {
        return;
    }
    if (message->status == StatusAndDescriptionMessage::done) {
        textToSpeech.end();
    } else {
        textToSpeech.fail();
    }
}

void ServiceModeControllerNode::whenReceivedSpeechRecognitionResult(StatusAndDescriptionMessage::ConstPtr const& message
) {
    if (!speechReognition.hasStarted) {
        return;
    }
    if (message->status == StatusAndDescriptionMessage::done) {
        ROS_DEBUG("语音识别结果: %s", message->description.c_str());
        speechReognition.foundYes = message->description.find("是") != std::string::npos;
        speechReognition.foundNo = message->description.find("否") != std::string::npos;
        speechReognition.end();
    } else {
        speechReognition.fail();
    }
}

void ServiceModeControllerNode::whenReceivedTemperatureMeasurementResult(Float64Message::ConstPtr const& result) {
    if (!temperatureMeasurement.hasStarted) {
        return;
    }
    temperatureMeasurement.temperature = Temperature{result->data, UnitTemperature::celcius};
    temperatureMeasurement.end();
}

void ServiceModeControllerNode::whenReceivedVideoCallResult(StatusAndDescriptionMessage::ConstPtr const& result) {
    if (!videoCall.hasStarted) {
        return;
    }
    if (result->status == StatusAndDescriptionMessage::done) {
        videoCall.end();
    } else {
        videoCall.fail();
    }
}

void ServiceModeControllerNode::delegateVelocityControl(LinearSpeed forward) {
    velocityControl.start();
    ROS_DEBUG("将前进速度设置为 %s。", forward.toString().c_str());
    VelocityCommand velocityCommand{forward, 0_m_per_s, 0_deg_per_s};
    velocityControlRequestPublisher.publish(velocityCommand.toMessage());
}

void ServiceModeControllerNode::delegateObjectDetectionControl(ObjectDetectionControl behavior) {
    switch (behavior) {
    case ObjectDetectionControl::Start:
        medicineDetection.start();
        nodeTiming.addTimedTask(
            3_s,
            [this]() {
                if (medicineDetection.hasStarted && !medicineDetection.hasEnded && !medicineDetection.hasFailed) {
                    delegateObjectDetectionControl(ObjectDetectionControl::Stop);
                    ROS_ERROR("药品检测超时。");
                    medicineDetection.fail();
                }
            },
            "药品检测超时"
        );
        break;
    case ObjectDetectionControl::Stop:
        medicineDetection.end();
        break;
    }
    ROS_DEBUG("%s。", toString(behavior).c_str());
    medicineDetectionRequestPublisher.publish(createObjectDetectionControlMessage(behavior));
}

void ServiceModeControllerNode::delegateNavigation(NavigationGoal goal) {
    navigation.start();
    navigationClient.sendGoal(goal);
    navigationWaitingThread = std::thread([this]() {
        bool finishedBeforeTimeout{navigationClient.waitForResult(ros::Duration(120))};
        GoalState state{navigationClient.getState()};
        if (state == GoalState::SUCCEEDED) {
            ROS_INFO("导航成功。");
            navigation.end();
        } else {
            ROS_INFO("导航因为某种原因失败。");
            navigation.fail();
        }
        if (!finishedBeforeTimeout) {
            ROS_INFO("导航因为超时而失败。");
            navigationClient.cancelGoal();
            navigation.fail();
        }
    });
}

void ServiceModeControllerNode::delegateControlingRobotManipulator(ManipulatorControl const& plan) {
    manipulatorControl.start();
    ROS_DEBUG("将把机械臂状态设置为 %s。", plan.toString().c_str());
    manipulatiorControlRequestPublisher.publish(plan.toMessage());
}

void ServiceModeControllerNode::delegateObjectGrasping(Coordinate coordinate) {
    medicineGrasp.start();
    ROS_DEBUG("将抓取位于 %s 的物体。", coordinate.toString().c_str());
    medicineGraspRequestPublisher.publish(coordinate.toMessage());
}

void ServiceModeControllerNode::delegateTextToSpeech(std::string content) {
    textToSpeech.start();
    ROS_DEBUG("将以下内容转为语音：%s.", content.c_str());
    StringMessage message;
    message.data = content;
    textToSpeechRequestPublisher.publish<StringMessage>(message);
}

void ServiceModeControllerNode::delegateSpeechRecognition(Duration duration) {
    speechReognition.start();
    StringMessage request;
    request.data = std::to_string(duration.getBaseUnitValue());
    speechRecognitionRequestPublisher.publish<StringMessage>(request);
}

void ServiceModeControllerNode::delegateTemperatureMeasurement() {
    temperatureMeasurement.start();
    temperatureMeasurementRequestPublisher.publish(EmptyMessage{});
}

void ServiceModeControllerNode::delegateVideoCall() {
    videoCall.start();
    videoCallRequestPublisher.publish(EmptyMessage{});
}

StringMessage ServiceModeControllerNode::createWaypointMessage(std::string name) {
    StringMessage message{};
    message.data = std::move(name);
    return message;
}

StringMessage ServiceModeControllerNode::createObjectDetectionControlMessage(ObjectDetectionControl behavior) {
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

void ServiceModeControllerNode::displayDetectedObjects(ObjectDetectionResultMessasge::ConstPtr const& coordinates_ptr) {
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

std::string ServiceModeControllerNode::toString(ObjectDetectionControl behavior) {
    switch (behavior) {
    case ObjectDetectionControl::Start:
        return "开启物体检测";
    case ObjectDetectionControl::Stop:
        return "停止物体检测";
    }
    return "";
}

std::string ServiceModeControllerNode::toString(TaskState taskState) {
    switch (taskState) {
    case TaskState::WaitingForPositionInitialization:
        return "等待校准初始位置";
    case TaskState::ReadyToPerformTasks:
        return "准备就绪";
    case TaskState::GoingToPharmacy:
        return "前往药房中";
    case TaskState::DetectingMedicine:
        return "检测药品位置中";
    case TaskState::GraspingMedicine:
        return "抓取药品中";
    case TaskState::GoingToPatient:
        return "前往患者所在位置中";
    case TaskState::DroppingMedicine:
        return "投递药品中";
    case TaskState::SteppingBackward:
        return "后撤中";
    case TaskState::RetractingTheArm:
        return "收回机械臂中";
    case TaskState::HaveFinishedPreviousTask:
        return "已完成上一个任务";
    case TaskState::GoingToBaseStation:
        return "前往充电基站";
    case TaskState::WaypointUnreachable:
        return "航点不可达";
    case TaskState::AskingMeasuringTemperature:
        return "提示患者测量体温";
    case TaskState::AskingIfVideoNeeded:
        return "询问患者是否需要视频通话";
    case TaskState::VideoCommunicating:
        return "视频通话中";
    case TaskState::MeasuringTemperature:
        return "测量患者体温中";
    case TaskState::ConfirmPatientRequest:
        return "确认患者语音";
    case TaskState::SpeechRecognitionFailed:
        return "语音识别失败";
    case TaskState::GiveUpCurrentTask:
        return "放弃当前任务";
    case TaskState::SpeakingPrescription:
        return "播放医嘱中";
    case TaskState::AskingIfGrabbedMedicine:
        return "询问患者是否已准备好伸手取药";
    case TaskState::WaitingForGrabbingMedicine:
        return "等待患者伸手取药";
    case TaskState::MedicineDetectionFailed:
        return "药品检测失败";
    case TaskState::VideoCallFailed:
        return "视频通话失败";
    }
    return "";
}
}  // namespace Nodes
}  // namespace xiaohu_robot