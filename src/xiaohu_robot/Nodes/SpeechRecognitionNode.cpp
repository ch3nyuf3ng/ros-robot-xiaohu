#include "xiaohu_robot/Nodes/SpeechRecognitionNode.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "xfyun_mod/msp_cmn.h"
#include "xfyun_mod/msp_errors.h"
#include "xfyun_mod/msp_types.h"
#include "xfyun_mod/qisr.h"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/Exceptions.hpp"
#include <chrono>
#include <clocale>
#include <exception>
#include <iostream>
#include <ostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");
    ros::init(argc, argv, CommonConfigs::SpeechRecognitionNodeName);
    SpeechRecognitionNode speechRecognitionNode{{}};
    speechRecognitionNode.run();
    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
SpeechRecognitionNode::SpeechRecognitionNode(SpeechRecognitionNode::Configs configs):
    nodeHandle(configs.nodeBasicConfigs.nodeNamespace),
    speechRecognitionRequestSubsriber{nodeHandle.subscribe<StringMessage>(
        configs.speechRecognitionRequestTopic,
        configs.nodeBasicConfigs.messageBufferSize,
        &SpeechRecognitionNode::whenReceivedSpeechRecognitionRequest,
        this
    )},
    speechRecognitionResultPublisher{nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.speechRecognotionResultTopic, configs.nodeBasicConfigs.messageBufferSize
    )},
    configs{std::move(configs)} {
    int errorCode = MSPLogin(NULL, NULL, this->configs.xfyunApiLoginParams.c_str());
    if (errorCode != MSP_SUCCESS) {
        printMessageThenThrowRuntimeError("讯飞云 API 登录失败");
    }
    try {
        uploadGrammar();
        std::ostringstream oss;
        oss << "engine_type = local, ";
        oss << "asr_threshold = 10, ";
        oss << "asr_res_path = fo|" << configs.asrParams.asrResourcePath << ", ";
        oss << "grm_build_path = " << configs.asrParams.asrGrammarBuildPath << ", ";
        oss << "local_grammar = " << grammarHandle.grammarId << ", ";
        oss << "text_encoding = UTF-8, ";
        oss << "result_encoding = UTF-8";
        asrSessionParams = oss.str();
    } catch (std::runtime_error const& e) {
        std::cerr << "语音识别节点初始化失败。" << std::endl;
        logout();
    }
    std::cout << "语音识别节点已启动。" << std::endl;
}

SpeechRecognitionNode::~SpeechRecognitionNode() {
    if (processThread.joinable()) {
        processThread.join();
    }
    if (!sessionId.empty()) {
        int errorCode{QISRSessionEnd(sessionId.c_str(), "")};
        if (errorCode != MSP_SUCCESS) {
            ROS_ERROR("会话结束失败，错误码: %d.\n", errorCode);
        }
    }
    logout();
    std::cout << "语音识别节点已退出。" << std::endl;
}

void SpeechRecognitionNode::run() {
    ros::spin();
}

void SpeechRecognitionNode::logout() {
    int errorCode = MSPLogout();
    if (errorCode != MSP_SUCCESS) {
        std::cerr << "合成音频账号注销失败，错误码：" << errorCode << std::endl;
    }
}

void SpeechRecognitionNode::endSession() {
    if (sessionId.empty())
        return;
    int endErrorCode{QISRSessionEnd(sessionId.c_str(), "")};
    if (endErrorCode != MSP_SUCCESS) {
        printMessageThenThrowRuntimeError("QISR End Session failed.");
    } else {
        std::cout << "语音识别会话结束。" << std::endl;
    }
}

void SpeechRecognitionNode::uploadGrammar() {
    int errorCode{MSP_SUCCESS};

    std::ostringstream oss;
    oss << "engine_type = local, ";
    oss << "sample_rate = " << 16000 << ", ";
    oss << "asr_res_path = fo|" << configs.asrParams.asrResourcePath << ", ";
    oss << "grm_build_path = " << configs.asrParams.asrGrammarBuildPath << ", ";
    oss << "asr_threshold = " << configs.asrParams.asrThreshold;
    std::string buildGrammarParams{oss.str()};
    errorCode = QISRBuildGrammar(
        "bnf",
        configs.grammar.data(),
        configs.grammar.size(),
        buildGrammarParams.c_str(),
        &whenGrammarHasBuilt,
        &grammarHandle
    );

    while (!grammarHandle.hasBuilt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

int SpeechRecognitionNode::whenGrammarHasBuilt(int errorCode, char const* grammarId, void* userData) {
    GrammarHandle* grammarHandle = static_cast<GrammarHandle*>(userData);
    if (errorCode == MSP_SUCCESS && grammarId != nullptr) {
        grammarHandle->grammarId = grammarId;
        grammarHandle->hasBuilt = true;
    } else {
        printMessageThenThrowRuntimeError("语法上传失败。");
    }
    return 0;
}

void SpeechRecognitionNode::whenReceivedSpeechRecognitionRequest(StringMessage::ConstPtr const& request) {
    if (isThreadEnded) {
        processThread.join();
        isThreadEnded = false;
    }

    if (!processThread.joinable()) {
        auto durationRepr{std::stol(request->data)};
        std::chrono::seconds duration{durationRepr};
        soundplayClient.play(1);
        std::this_thread::sleep_for(std::chrono::seconds(1));
        processThread = std::thread([duration, this]() {
            try {
                StatusAndDescriptionMessage result;
                result.status = StatusAndDescriptionMessage::done;
                result.description = speechRecognitionFromMicrophone(duration);
                ROS_INFO("语音识别成功，内容为：%s", result.description.c_str());
                speechRecognitionResultPublisher.publish(result);
            } catch (std::exception const& e) {
                StatusAndDescriptionMessage result;
                result.status = StatusAndDescriptionMessage::failed;
                result.description = e.what();
                ROS_ERROR("语音识别失败，原因是：%s", e.what());
                speechRecognitionResultPublisher.publish(result);
            }
            isThreadEnded = true;
        });
    } else {
        StatusAndDescriptionMessage result;
        result.status = StatusAndDescriptionMessage::cancelled;
        result.description = "正在处理上一个语音识别任务。";
        ROS_WARN("%s", result.description.c_str());
        speechRecognitionResultPublisher.publish(result);
    }
}

std::string SpeechRecognitionNode::speechRecognitionFromMicrophone(std::chrono::seconds duration) {
    int errorCode{MSP_SUCCESS};
    sessionId = QISRSessionBegin(nullptr, asrSessionParams.c_str(), &errorCode);
    if (errorCode != MSP_SUCCESS) {
        printMessageThenThrowRuntimeError("QISR Session begin failed.");
    } else {
        ROS_INFO("语音识别会话开始。");
    }

    RecorderContext recorderContext{};
    Recorder recorder{whenPeriodDataIsReady(recorderContext), whenRecorderStopped(recorderContext), WAVEFORMATEX{}};
    recorder.start();
    auto startTime{std::chrono::high_resolution_clock::now()};
    auto currentTime{startTime};
    while (!recorderContext.hasGotResult
           && std::chrono::duration_cast<std::chrono::seconds>(currentTime - startTime) < duration)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        recorder.checkThreadsException();
        currentTime = std::chrono::high_resolution_clock::now();
    }
    recorder.stop();
    while (!recorderContext.hasGotResult) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!recorderContext.result.empty()) {
        std::size_t index{recorderContext.result.find("input=")};
        std::string finalResult(recorderContext.result, index + 6);
        return finalResult;
    } else {
        printMessageThenThrowRuntimeError("没有检测到命令词。");
    }
}

Recorder::DataCallback SpeechRecognitionNode::whenPeriodDataIsReady(RecorderContext& context) {
    return [this, &context](std::vector<char> const& data) {
        int audioStatus{context.isFirstBlock ? MSP_AUDIO_SAMPLE_FIRST : MSP_AUDIO_SAMPLE_CONTINUE};
        int endPointStatus{MSP_EP_LOOKING_FOR_SPEECH};
        int recognitionStatus{MSP_REC_STATUS_INCOMPLETE};
        int audioWriteErrorCode{QISRAudioWrite(
            sessionId.c_str(), data.data(), data.size(), audioStatus, &endPointStatus, &recognitionStatus
        )};
        if (audioWriteErrorCode != MSP_SUCCESS) {
            std::ostringstream oss;
            oss << "AudioWriteErrorCode: " << audioWriteErrorCode << "\n";
            oss << "EndPointStatus: " << endPointStatus << "\n";
            oss << "RecognitionStatus: " << recognitionStatus;
            printMessageThenThrowRuntimeError(oss.str());
        }
        if (recognitionStatus != MSP_REC_STATUS_SUCCESS) {
            return;
        }
        int getResultErrorCode{MSP_SUCCESS};
        char const* resultPointer{QISRGetResult(sessionId.c_str(), &recognitionStatus, 0, &getResultErrorCode)};
        if (getResultErrorCode != MSP_SUCCESS) {
            std::ostringstream oss;
            oss << "GetResultErrorCode: " << getResultErrorCode << "\n";
            oss << "RecognitionStatus: " << recognitionStatus;
            printMessageThenThrowRuntimeError(oss.str());
        }
        if (resultPointer) {
            context.hasGotResult = true;
            context.result = resultPointer;
        }
    };
}

Recorder::EndCallback SpeechRecognitionNode::whenRecorderStopped(RecorderContext& context) {
    return [this, &context]() {
        int endPointStatus{MSP_EP_LOOKING_FOR_SPEECH};
        int recognitionStatus{MSP_REC_STATUS_INCOMPLETE};
        int audioWriteErrorCode{
            QISRAudioWrite(sessionId.c_str(), nullptr, 0, MSP_AUDIO_SAMPLE_LAST, &endPointStatus, &recognitionStatus)
        };
        if (audioWriteErrorCode != MSP_SUCCESS) {
            std::ostringstream oss;
            oss << "AudioWriteErrorCode: " << audioWriteErrorCode << "\n";
            oss << "EndPointStatus: " << endPointStatus << "\n";
            oss << "RecognitionStatus: " << recognitionStatus;
            printMessageThenThrowRuntimeError(oss.str());
        }
        if (context.hasGotResult) {
            endSession();
            return;
        }
        int getResultStatus{MSP_REC_STATUS_SUCCESS};
        int getResultErrorCode{MSP_SUCCESS};
        while (getResultErrorCode == MSP_SUCCESS && getResultStatus != MSP_REC_STATUS_COMPLETE) {
            char const* resultPointer{QISRGetResult(sessionId.c_str(), &getResultStatus, 0, &getResultErrorCode)};
            if (getResultErrorCode != MSP_SUCCESS) {
                endSession();
                std::ostringstream oss;
                oss << "GetResultErrorCode: " << getResultErrorCode << "\n";
                oss << "RecognitionStatus: " << recognitionStatus;
                printMessageThenThrowRuntimeError(oss.str());
            }
            if (resultPointer) {
                context.result = resultPointer;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        context.hasGotResult = true;
        int endErrorCode{MSP_SUCCESS};
        endSession();
    };
}
}  // namespace Nodes
}  // namespace xiaohu_robot