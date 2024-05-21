#include "xiaohu_robot/Nodes/TaskControllerNode.hpp"
#include "ros/console.h"
#include "xiaohu_robot/Foundation/Coordinate.hpp"
#include "xiaohu_robot/Foundation/ManipulatorControl.hpp"
#include "xiaohu_robot/Foundation/Task.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/VelocityCommand.hpp"
#include "xiaohu_robot/InspectionTaskMessage.h"
#include "xiaohu_robot/MappingTaskMessage.h"
#include "xiaohu_robot/MedicineDeliveryTaskMessage.h"
#include <chrono>
#include <clocale>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");

    ros::init(argc, argv, "task_ctrl_node");
    TaskControllerNode taskControllerNode{TaskControllerNode::Configs{}};
    taskControllerNode.run();

    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
TaskControllerNode::TaskControllerNode(Configs configs):
    nodeHandle(configs.nodeBasicConfig.nodeNamespace),
    nodeTiming{configs.nodeBasicConfig.loopFrequency},
    initPosition{},
    baseStatePosition{},
    initPositionRequestSubscriber{nodeHandle.subscribe<CoordinateMessage>(
        configs.initPositionRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedInitPositionRequest,
        this
    )},
    initPositionResultSubscriber{nodeHandle.subscribe<StatusAndDescriptionMessage>(
        configs.initPositionResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedInitPositionResult,
        this
    )},
    taskState{},
    currentTaskLegacy{},
    legacyGeneralTasks{},
    legacyGeneralTaskRequestSubscriber{nodeHandle.subscribe<GeneralTaskMessage>(
        configs.legacyGeneralTasksRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedLegacyGeneralTaskRequest,
        this
    )},
    tasks{},
    inspectionTaskRequestSubscriber{nodeHandle.subscribe<InspectionTaskMessage>(
        configs.inspectionTaskRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedInspectionTaskRequest,
        this
    )},
    mappingTaskRequestSubscriber{nodeHandle.subscribe<MappingTaskMessage>(
        configs.mappingTaskRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedMappingTaskRequest,
        this
    )},
    medicineDeliveryRequestSubsriber{nodeHandle.subscribe<MedicineDeliveryTaskMessage>(
        configs.medicineDeliveryTaskRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedMedicineDeliveryTaskRequest,
        this
    )},
    velocityControl{},
    velocityControlRequestPublisher{nodeHandle.advertise<VelocityCommandMessage>(
        configs.velocityControlRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    manipulatorControl{},
    manipulatiorControlRequestPublisher{nodeHandle.advertise<ManipulatorControlMessage>(
        configs.manipulatorControlRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    waypointNavigation{},
    waypointNavigationRequestPublisher{nodeHandle.advertise<StringMessage>(
        configs.waypointNavigationRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    waypointNavigationResultSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        configs.waypointNavigationResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedNavigationResult,
        this
    )},
    waypointUnreachableTimes{0},
    obstacleClearing{},
    navigationClient{configs.coordinateNavigationTopic},
    textToSpeech{},
    textToSpeechRequestPublisher{
        nodeHandle.advertise<StringMessage>(configs.textToSpeechRequestTopic, configs.nodeBasicConfig.messageBufferSize)
    },
    textToSpeechResultSubscriber{nodeHandle.subscribe<StatusAndDescriptionMessage>(
        configs.textToSpeechResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedTextToSpeechResult,
        this
    )},
    speechReognition{},
    speechRecognitionRequestPublisher{nodeHandle.advertise<StringMessage>(
        configs.speechRecognitionRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    speechRecognitionResultSubscriber{nodeHandle.subscribe<StatusAndDescriptionMessage>(
        configs.speechRecognitionResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedSpeechRecognitionResult,
        this
    )},
    speechRecognitionContinuousFailureTimes{0},
    medicineDetection{},
    medicineDetectionRequestPublisher{nodeHandle.advertise<StringMessage>(
        configs.medcineDetectionRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    medicineDetectionResultSubscriber{nodeHandle.subscribe<ObjectDetectionResultMessasgePointer>(
        configs.medicineDetectionResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedMedicineDetectionResult,
        this
    )},
    medicineGrasp{},
    medicineGraspRequestPublisher{nodeHandle.advertise<CoordinateMessage>(
        configs.medicineGraspRequestTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    medicineGraspResultSubscriber{nodeHandle.subscribe<StringMessagePointer>(
        configs.medicineGraspResultTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TaskControllerNode::whenReceivedMedicineGraspResult,
        this
    )},
    soundPlayClient{},
    temperatureMeasurement{},
    configs{std::move(configs)} {
    std::cout << "任务控制器已启动。" << std::endl;
    std::this_thread::sleep_for(std::chrono::seconds(1));
}

TaskControllerNode::~TaskControllerNode() {
    std::cout << "任务控制器已退出。" << std::endl;
}

void TaskControllerNode::DelegationState::reset() {
    hasStarted = false;
    hasFailed = false;
    hasEnded = false;
};

void TaskControllerNode::DelegationState::start() {
    hasStarted = true;
}

void TaskControllerNode::DelegationState::end() {
    hasEnded = true;
}

void TaskControllerNode::DelegationState::fail() {
    hasFailed = true;
}

void TaskControllerNode::SpeechRecognitionContext::reset() {
    DelegationState::reset();
    foundYes = false;
    foundNo = false;
}

void TaskControllerNode::MedicineDetectionAndGraspContext::reset() {
    DelegationState::reset();
    medicinePosition = Coordinate{};
}

void TaskControllerNode::run() {
    ros::Rate loopRate{configs.nodeBasicConfig.loopFrequency};
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
        case TaskState::MeasuringTemperature:
            measuringTemperature();
            break;
        case TaskState::AskingIfVideoNeeded:
            askIfVideoNeeded();
            break;
        case TaskState::VideoCommunicating:
            videoCommunicating();
            break;
        case TaskState::HasMeasuredTemperature:
            hasMeasuredTemperature();
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
        }
        checkNavigationState();
        ros::spinOnce();
        if (!taskState.stateTransferring) {
            nodeTiming.increment();
        } else {
            taskState.stateTransferring = false;
        }
        loopRate.sleep();
    }
}

MappingTask& TaskControllerNode::getCurrentMappingTask() {
    return dynamic_cast<MappingTask&>(getCurrentTask());
}

InspectionTask& TaskControllerNode::getCurrentInspectionTask() {
    return dynamic_cast<InspectionTask&>(getCurrentTask());
}

MedicineDeliveryTask& TaskControllerNode::getCurrentMedicineDeliveryTask() {
    return dynamic_cast<MedicineDeliveryTask&>(getCurrentTask());
}

bool TaskControllerNode::isCurrentTaskLegacy() const {
    return currentTaskLegacy.front();
}

SpecificTask& TaskControllerNode::getCurrentTask() {
    return *tasks.front();
}

LegacyGeneralTask& TaskControllerNode::getCurrentLegacyTask() {
    return legacyGeneralTasks.front();
}

TaskControllerNode::TaskState TaskControllerNode::getCurrentTaskState() const {
    return taskState.currentTaskState;
}

TaskControllerNode::TaskState TaskControllerNode::getPreviousTaskState() const {
    return taskState.previousTaskState;
}

void TaskControllerNode::transferCurrentTaskStateTo(TaskState next) {
    taskState.previousTaskState = taskState.currentTaskState;
    taskState.currentTaskState = next;
    taskState.stateTransferring = true;

    ROS_INFO(
        "任务状态: %s -> %s",
        toString(taskState.previousTaskState).c_str(),
        toString(taskState.currentTaskState).c_str()
    );
}

void TaskControllerNode::setPreviousTaskState(TaskState previous) {
    taskState.previousTaskState = previous;
}

void TaskControllerNode::checkNavigationState() {
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

void TaskControllerNode::startInitialPositionCalibration() {
    if (!textToSpeech.hasStarted && !initPosition.hasStarted) {
        initPosition.hasStarted = true;
        std::string hint{"请确认机器人初始位置位于充电基站。"};
        ROS_INFO("%s", hint.c_str());
        delegateTextToSpeech(hint);
    } else if (textToSpeech.hasEnded && initPosition.hasEnded) {
        transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
        textToSpeech.reset();
        initPosition.reset();
    }
}

void TaskControllerNode::readyToPerformTasks() {
    if (currentTaskLegacy.empty()) {
        return;
    }
    switch (isCurrentTaskLegacy() ? getCurrentLegacyTask().getTaskType() : getCurrentTask().getTaskType()) {
    case TaskType::Mapping:
        throw UnsupportedTaskException{};
    case TaskType::Inspection:
        transferCurrentTaskStateTo(TaskState::GoingToPatient);
        break;
    case TaskType::MedicineDelivery:
        transferCurrentTaskStateTo(TaskState::GoingToPharmacy);
        break;
    }
}

void TaskControllerNode::goToPharmacy() {
    if (isCurrentTaskLegacy()) {
        if (!waypointNavigation.hasStarted) {
            delegateNavigatingToWaypoint(getCurrentLegacyTask().pharmacy.c_str());
        } else if (waypointNavigation.hasEnded) {
            transferCurrentTaskStateTo(TaskState::DetectingMedicine);
            waypointNavigation.reset();
        }
    } else {
        if (!navigation.hasStarted) {
            navigation.start();
            navigationClient.sendGoal(getCurrentMedicineDeliveryTask().pharmacyPosition.toNavigationGoal());
        } else if (navigation.hasEnded) {
            transferCurrentTaskStateTo(TaskState::DetectingMedicine);
            navigation.reset();
        }
    }
}

void TaskControllerNode::detectMedicine() {
    if (!medicineDetection.hasStarted) {
        delegateObjectDetectionControl(ObjectDetectionControl::Start);
    } else if (medicineDetection.hasEnded) {
        medicineGrasp.medicinePosition = medicineDetection.medicinePosition;
        delegateObjectDetectionControl(ObjectDetectionControl::Stop);
        transferCurrentTaskStateTo(TaskState::GraspingMedicine);
        medicineDetection.reset();
    }
}

void TaskControllerNode::graspMedicine() {
    if (!medicineGrasp.hasStarted) {
        delegateObjectGrasping(medicineGrasp.medicinePosition);
    } else if (medicineGrasp.hasEnded) {
        transferCurrentTaskStateTo(TaskState::GoingToPatient);
        medicineGrasp.reset();
    }
}

void TaskControllerNode::goToPatient() {
    if (isCurrentTaskLegacy()) {
        if (!waypointNavigation.hasStarted) {
            delegateNavigatingToWaypoint(getCurrentLegacyTask().patient);
        } else if (waypointNavigation.hasEnded) {
            switch (getCurrentLegacyTask().getTaskType()) {
            case TaskType::Mapping:
                throw UnsupportedTaskException{};
            case TaskType::Inspection:
                transferCurrentTaskStateTo(TaskState::MeasuringTemperature);
                break;
            case TaskType::MedicineDelivery:
                transferCurrentTaskStateTo(TaskState::SpeakingPrescription);
                break;
            }
            waypointNavigation.reset();
        }
    } else {
        if (!navigation.hasStarted) {
            switch (getCurrentTask().getTaskType()) {
            case TaskType::Mapping:
                throw UnsupportedTaskException();
            case TaskType::Inspection:
                navigation.start();
                navigationClient.sendGoal(getCurrentInspectionTask().patientPosition.toNavigationGoal());
                break;
            case TaskType::MedicineDelivery:
                navigation.start();
                navigationClient.sendGoal(getCurrentMedicineDeliveryTask().patientPosition.toNavigationGoal());
                break;
            }
        } else if (navigation.hasEnded) {
            switch (getCurrentTask().getTaskType()) {
            case TaskType::Mapping:
                throw UnsupportedTaskException{};
            case TaskType::Inspection:
                transferCurrentTaskStateTo(TaskState::MeasuringTemperature);
                break;
            case TaskType::MedicineDelivery:
                transferCurrentTaskStateTo(TaskState::SpeakingPrescription);
                break;
            }
            navigation.reset();
        }
    }
}

void TaskControllerNode::speakPrescription() {
    if (isCurrentTaskLegacy()) {
        if (!textToSpeech.hasStarted) {
            delegateTextToSpeech(getCurrentLegacyTask().prescription + "……请您伸手取药。");
        } else if (textToSpeech.hasEnded) {
            transferCurrentTaskStateTo(TaskState::AskingIfGrabbedMedicine);
            textToSpeech.reset();
        }
    } else {
        if (!textToSpeech.hasStarted) {
            std::ostringstream oss;
            oss << "患者" << getCurrentMedicineDeliveryTask().patientName << "，您有一份药品待取：";
            oss << getCurrentMedicineDeliveryTask().prescription << "……";
            oss << "请您伸手取药。";
            delegateTextToSpeech(oss.str());
        } else if (textToSpeech.hasEnded) {
            transferCurrentTaskStateTo(TaskState::AskingIfGrabbedMedicine);
            textToSpeech.reset();
        }
    }
}

void TaskControllerNode::askIfGrabbedMedicine() {
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
        speechRecognitionContinuousFailureTimes = 0;
        textToSpeech.reset();
        speechReognition.reset();
    } else if (textToSpeech.hasEnded && speechReognition.hasFailed) {
        transferCurrentTaskStateTo(TaskState::SpeechRecognitionFailed);
        textToSpeech.reset();
        speechReognition.reset();
    }
}

void TaskControllerNode::waitForGrabbingMedicine() {
    if (!textToSpeech.hasStarted && !waiting.hasStarted) {
        delegateTextToSpeech("好的，我会再等待您十秒钟。");
        waiting.hasStarted = true;
        nodeTiming.addTimedTask(10_s, [this]() { waiting.hasEnded = true; }, "等待患者抓取药品结束");
    } else if (textToSpeech.hasEnded && waiting.hasEnded) {
        transferCurrentTaskStateTo(TaskState::AskingIfGrabbedMedicine);
        textToSpeech.reset();
        waiting.reset();
    }
}

void TaskControllerNode::dropMedicine() {
    if (!manipulatorControl.hasStarted) {
        delegateControlingRobotManipulator(GripperControl{15_cm});
        nodeTiming.addTimedTask(3_s, [this]() { manipulatorControl.hasEnded = true; }, "结束张开机械手");
    } else if (manipulatorControl.hasEnded) {
        transferCurrentTaskStateTo(TaskState::SteppingBackward);
        manipulatorControl.reset();
    }
}

void TaskControllerNode::stepBackward() {
    if (!velocityControl.hasStarted) {
        delegateVelocityControl(-0.1_m_per_s);
        nodeTiming.addTimedTask(
            5_s,
            [this]() {
                delegateVelocityControl(0_m_per_s);
                velocityControl.hasEnded = true;
            },
            "停止后退"
        );
    } else if (velocityControl.hasEnded) {
        transferCurrentTaskStateTo(TaskState::RetractingTheArm);
        velocityControl.reset();
    }
}

void TaskControllerNode::retractManipulaor() {
    if (!manipulatorControl.hasStarted) {
        delegateControlingRobotManipulator(
            MultiPartControl{std::make_unique<ArmControl>(0_cm), std::make_unique<GripperControl>(0_cm)}
        );
        nodeTiming.addTimedTask(5_s, [this]() { manipulatorControl.hasEnded = true; }, "停止回收机械臂");
    } else if (manipulatorControl.hasEnded) {
        transferCurrentTaskStateTo(TaskState::HaveFinishedPreviousTask);
        manipulatorControl.reset();
    }
}

void TaskControllerNode::haveFinishedPreviousTask() {
    if (isCurrentTaskLegacy()) {
        switch (getCurrentLegacyTask().getTaskType()) {
        case TaskType::Mapping:
            throw UnsupportedTaskException{};
        case TaskType::Inspection:
            ROS_INFO("已完成巡诊任务 (%s)。", getCurrentLegacyTask().patient.c_str());
            break;
        case TaskType::MedicineDelivery:
            ROS_INFO(
                "已完成送药任务 (%s -> %s)。",
                getCurrentLegacyTask().pharmacy.c_str(),
                getCurrentLegacyTask().patient.c_str()
            );
            break;
        }
        legacyGeneralTasks.pop_front();
    } else {
        ROS_INFO("已完成%s。", getCurrentTask().toString().c_str());
        tasks.pop_front();
    }
    currentTaskLegacy.pop_front();
    showRemainedTasksCount();
    if (!currentTaskLegacy.empty()) {
        transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
    } else {
        transferCurrentTaskStateTo(TaskState::GoingToBaseStation);
    }
}

void TaskControllerNode::askIfVideoNeeded() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("请问您是否需要与医生视频通话？");
    } else if (textToSpeech.hasEnded && !speechReognition.hasStarted) {
        delegateSpeechRecognition(10_s);
    } else if (speechReognition.hasFailed) {
        transferCurrentTaskStateTo(TaskState::SpeechRecognitionFailed);
        textToSpeech.reset();
    } else if (speechReognition.hasEnded) {
        transferCurrentTaskStateTo(TaskState::ConfirmPatientRequest);
        speechRecognitionContinuousFailureTimes = 0;
        textToSpeech.reset();
    }
}

void TaskControllerNode::speechRecognitionFailed() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("抱歉，我没有听清，请再试一次。您可以说“是”，或者“否”。");
        speechRecognitionContinuousFailureTimes++;
    } else if (textToSpeech.hasEnded) {
        if (speechRecognitionContinuousFailureTimes < 3) {
            transferCurrentTaskStateTo(getPreviousTaskState());
        } else {
            transferCurrentTaskStateTo(TaskState::GiveUpCurrentTask);
        }
        textToSpeech.reset();
    }
}

void TaskControllerNode::giveUpCurrentTask() {
    if (!textToSpeech.hasStarted && !exceptionHandling.hasStarted) {
        delegateTextToSpeech("重复出现异常。任务中断。");
        exceptionHandling.hasStarted = true;
    } else if (textToSpeech.hasEnded && exceptionHandling.hasEnded) {
        transferCurrentTaskStateTo(getPreviousTaskState());
        textToSpeech.reset();
        exceptionHandling.reset();
    }
}

void TaskControllerNode::confirmPatientRequest() {
    if (!textToSpeech.hasStarted) {
        if (speechReognition.foundYes) {
            delegateTextToSpeech("好的，已为您安排视频通话。");
        } else {
            delegateTextToSpeech("好的，已取消视频通话。");
        }
    } else if (textToSpeech.hasEnded) {
        if (speechReognition.foundYes) {
            transferCurrentTaskStateTo(TaskState::VideoCommunicating);
        } else {
            transferCurrentTaskStateTo(TaskState::HaveFinishedPreviousTask);
        }
        speechReognition.reset();
        textToSpeech.reset();
    }
}

void TaskControllerNode::videoCommunicating() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("语音通话暂未实现。");
    } else if (textToSpeech.hasEnded) {
        transferCurrentTaskStateTo(TaskState::HaveFinishedPreviousTask);
        textToSpeech.reset();
    }
}

void TaskControllerNode::goToBaseStation() {
    if (currentTaskLegacy.empty()) {
        if (!navigation.hasStarted) {
            navigation.start();
            navigationClient.sendGoal(baseStatePosition.toNavigationGoal());
        } else if (navigation.hasEnded) {
            transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
            navigation.reset();
        } else if (navigation.hasFailed) {
            transferCurrentTaskStateTo(TaskState::WaypointUnreachable);
            navigation.reset();
        }
    } else {
        if (!navigation.hasStarted) {
            transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
        } else {
            if (navigation.hasEnded) {
                transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
                navigation.reset();
            } else {
                if (!navigation.hasFailed) {
                    navigationClient.cancelGoal();
                } else {
                    transferCurrentTaskStateTo(TaskState::ReadyToPerformTasks);
                    navigation.reset();
                }
            }
        }
    }
}

void TaskControllerNode::waypointUnreachable() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("无法前往下一处航点，请尝试移除周围的障碍物，机器人会在 20 秒内重试。");
    } else if (textToSpeech.hasEnded && !obstacleClearing.hasStarted) {
        obstacleClearing.hasStarted = true;
        nodeTiming.addTimedTask(20_s, [this]() { obstacleClearing.hasEnded = true; }, "等待清除障碍物结束");
    } else if (textToSpeech.hasEnded && obstacleClearing.hasEnded) {
        transferCurrentTaskStateTo(getPreviousTaskState());
        textToSpeech.reset();
        obstacleClearing.reset();
    }
}

void TaskControllerNode::measuringTemperature() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("请将手腕对准温度传感器测量体温。");
    } else if (textToSpeech.hasEnded && !temperatureMeasurement.hasStarted) {
        temperatureMeasurement.hasStarted = true;
        nodeTiming.addTimedTask(3_s, [this]() { temperatureMeasurement.hasEnded = true; }, "测温结束");
    } else if (textToSpeech.hasEnded && temperatureMeasurement.hasEnded) {
        transferCurrentTaskStateTo(TaskState::HasMeasuredTemperature);
        textToSpeech.reset();
        temperatureMeasurement.reset();
    }
}

void TaskControllerNode::hasMeasuredTemperature() {
    if (!textToSpeech.hasStarted) {
        delegateTextToSpeech("测温结束，您的体温正常。");
    } else if (textToSpeech.hasEnded) {
        transferCurrentTaskStateTo(TaskState::AskingIfVideoNeeded);
        textToSpeech.reset();
    }
}

void TaskControllerNode::showRemainedTasksCount() const {
    if (currentTaskLegacy.empty())
        ROS_INFO("没有任务可被执行。");
    else {
        ROS_INFO("剩余任务数量: %zd", legacyGeneralTasks.size() + tasks.size());
    }
}

void TaskControllerNode::showTiming() const {
    ROS_INFO("当前状态计时: %s", nodeTiming.getCurrentTiming().toString().c_str());
}

void TaskControllerNode::displayInitializationResult() const {
    ROS_INFO("任务控制器已初始化。");
}

void TaskControllerNode::whenReceivedInitPositionRequest(CoordinateMessagePointer message) {
    baseStatePosition = Coordinate{message};
    ROS_INFO("充电基站坐标已初始化。");
    std::cout << baseStatePosition.toString() << std::endl;
}

void TaskControllerNode::whenReceivedInitPositionResult(StatusAndDescriptionMessagePointer result) {
    if (!initPosition.hasStarted) {
        return;
    }
    if (result->status == StatusAndDescriptionMessage::done) {
        initPosition.hasEnded = true;
    }
}

void TaskControllerNode::whenReceivedMappingTaskRequest(MappingTaskMessagePointer message) {
    tasks.emplace_back(std::make_unique<MappingTask>(message));
    currentTaskLegacy.emplace_back(false);
}

void TaskControllerNode::whenReceivedInspectionTaskRequest(InspectionTaskMessagePointer message) {
    tasks.emplace_back(std::make_unique<InspectionTask>(message));
    currentTaskLegacy.emplace_back(false);
}

void TaskControllerNode::whenReceivedMedicineDeliveryTaskRequest(MedicineDeliveryTaskMessagePointer message) {
    tasks.emplace_back(std::make_unique<MedicineDeliveryTask>(message));
    currentTaskLegacy.emplace_back(false);
}

void TaskControllerNode::whenReceivedMedicineDetectionResult(ObjectDetectionResultMessasgePointer coordinates) {
    if (!medicineDetection.hasStarted) {
        return;
    }
    displayDetectedObjects(coordinates);
    constexpr std::size_t nearest_object_index{0};
    medicineDetection.medicinePosition = Coordinate{
        coordinates->x[nearest_object_index], coordinates->y[nearest_object_index], coordinates->z[nearest_object_index]
    };
    medicineDetection.end();
}

void TaskControllerNode::whenReceivedNavigationResult(StringMessagePointer message) {
    if (!waypointNavigation.hasStarted) {
        return;
    }
    if (message->data == "done") {
        waypointNavigation.end();
    } else {
        waypointNavigation.fail();
    }
}

void TaskControllerNode::whenReceivedMedicineGraspResult(StringMessagePointer message) {
    if (!medicineGrasp.hasStarted) {
        return;
    }
    if (message->data == "done") {
        medicineGrasp.end();
    } else {
        medicineGrasp.fail();
    }
}

void TaskControllerNode::whenReceivedLegacyGeneralTaskRequest(GeneralTaskMessagePointer message) {
    legacyGeneralTasks.emplace_back(LegacyGeneralTask{message});
    currentTaskLegacy.emplace_back(true);
    ROS_INFO("收到一个任务:\n%s", legacyGeneralTasks.back().toString().c_str());
}

void TaskControllerNode::whenReceivedStateControlCommand(StringMessagePointer message) {
    if (message->data == "clear") {
        while (legacyGeneralTasks.size() > 1) {
            legacyGeneralTasks.pop_back();
        }
        ROS_INFO("已取消当前任务，并清除剩余任务。");
        transferCurrentTaskStateTo(TaskState::GiveUpCurrentTask);
    }
}

void TaskControllerNode::whenReceivedTextToSpeechResult(StatusAndDescriptionMessagePointer message) {
    if (!textToSpeech.hasStarted) {
        return;
    }
    if (message->status == StatusAndDescriptionMessage::done) {
        textToSpeech.end();
    } else {
        textToSpeech.fail();
    }
}

void TaskControllerNode::whenReceivedSpeechRecognitionResult(StatusAndDescriptionMessagePointer message) {
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

void TaskControllerNode::delegateVelocityControl(LinearSpeed forward) {
    velocityControl.hasStarted = true;
    ROS_DEBUG("将前进速度设置为 %s。", forward.toString().c_str());
    VelocityCommand velocityCommand{forward, 0_m_per_s, 0_deg_per_s};
    velocityControlRequestPublisher.publish(velocityCommand.toMessage());
}

void TaskControllerNode::delegateObjectDetectionControl(ObjectDetectionControl behavior) {
    switch (behavior) {
    case ObjectDetectionControl::Start:
        medicineDetection.start();
        break;
    case ObjectDetectionControl::Stop:
        medicineDetection.end();
        break;
    }
    ROS_DEBUG("%s。", toString(behavior).c_str());
    medicineDetectionRequestPublisher.publish(createObjectDetectionControlMessage(behavior));
}

void TaskControllerNode::delegateNavigatingToWaypoint(std::string waypointName) {
    waypointNavigation.hasStarted = true;
    ROS_DEBUG("将导航至 %s。", waypointName.c_str());
    waypointNavigationRequestPublisher.publish(createWaypointMessage(std::move(waypointName)));
}

void TaskControllerNode::delegateControlingRobotManipulator(ManipulatorControl const& plan) {
    manipulatorControl.hasStarted = true;
    ROS_DEBUG("将把机械臂状态设置为 %s。", plan.toString().c_str());
    manipulatiorControlRequestPublisher.publish(plan.toMessage());
}

void TaskControllerNode::delegateObjectGrasping(Coordinate coordinate) {
    medicineGrasp.hasStarted = true;
    ROS_DEBUG("将抓取位于 %s 的物体。", coordinate.toString().c_str());
    medicineGraspRequestPublisher.publish(coordinate.toMessage());
}

void TaskControllerNode::delegateTextToSpeech(std::string content) {
    textToSpeech.hasStarted = true;
    ROS_DEBUG("将以下内容转为语音：%s.", content.c_str());
    StringMessage message;
    message.data = content;
    textToSpeechRequestPublisher.publish<StringMessage>(message);
}

void TaskControllerNode::delegateSpeechRecognition(Duration duration) {
    speechReognition.hasStarted = true;
    StringMessage request;
    request.data = std::to_string(duration.getBaseUnitValue());
    speechRecognitionRequestPublisher.publish<StringMessage>(request);
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
        return "开启物体检测";
    case ObjectDetectionControl::Stop:
        return "停止物体检测";
    }
    return "";
}

std::string TaskControllerNode::toString(TaskState taskState) {
    switch (taskState) {
    case TaskState::CalibratingInitialPosition:
        return "校准初始位置中";
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
    case TaskState::MeasuringTemperature:
        return "测量体温中";
    case TaskState::AskingIfVideoNeeded:
        return "询问患者是否需要视频通话";
    case TaskState::VideoCommunicating:
        return "视频通话中";
    case TaskState::HasMeasuredTemperature:
        return "已测量患者体温";
    case TaskState::ConfirmPatientRequest:
        return "确认患者语音";
    case TaskState::SpeechRecognitionFailed:
        return "语音识别失败。";
    case TaskState::GiveUpCurrentTask:
        return "放弃当前任务";
    case TaskState::SpeakingPrescription:
        return "播放医嘱中";
    case TaskState::AskingIfGrabbedMedicine:
        return "询问患者是由已抓取药品";
    case TaskState::WaitingForGrabbingMedicine:
        return "等待患者抓取药品";
    }
    return "";
}
}  // namespace Nodes
}  // namespace xiaohu_robot