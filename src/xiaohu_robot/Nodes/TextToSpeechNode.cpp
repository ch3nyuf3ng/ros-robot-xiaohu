#include "xiaohu_robot/Nodes/TextToSpeechNode.hpp"
#include "ros/console.h"
#include "ros/init.h"
#include "xfyun_mod/msp_cmn.h"
#include "xfyun_mod/msp_errors.h"
#include "xfyun_mod/qtts.h"
#include "xiaohu_robot/Foundation/Audio.hpp"
#include "xiaohu_robot/Foundation/File.hpp"
#include <chrono>
#include <stdexcept>
#include <thread>

int main(int argc, char* argv[]) {
    using namespace xiaohu_robot;
    std::setlocale(LC_ALL, "zh_CN.utf8");

    ros::init(argc, argv, "tts_node");
    TextToSpeechNode textToSpeechNode{TextToSpeechNode::Configs{}};
    textToSpeechNode.run();

    return 0;
}

namespace xiaohu_robot {
inline namespace Nodes {
TextToSpeechNode::TextToSpeechNode(TextToSpeechNode::Configs configs):
    nodeHandle(configs.nodeBasicConfig.nodeNamespace),
    textToSpeechRequestSubscriber{nodeHandle.subscribe<StringMessage>(
        configs.textToSpeechRequestTopic,
        configs.nodeBasicConfig.messageBufferSize,
        &TextToSpeechNode::whenReceivedTextToSpeechRequest,
        this
    )},
    textToSpeechResultPublisher{nodeHandle.advertise<StatusAndDescriptionMessage>(
        configs.textToSpeechResultTopic, configs.nodeBasicConfig.messageBufferSize
    )},
    configs{std::move(configs)} {
    int errorCode = MSPLogin(NULL, NULL, this->configs.loginParams.c_str());
    if (errorCode != MSP_SUCCESS) {
        throw std::runtime_error("合成音频账号登录失败");
    }
    std::cout << "语音合成节点已启动。" << std::endl;
}

TextToSpeechNode::~TextToSpeechNode() {
    if (processThread.joinable()) {
        processThread.join();
    }
    if (!sessionId.empty()) {
        int errorCode{QTTSSessionEnd(sessionId.c_str(), "Normal")};
        if (errorCode != MSP_SUCCESS) {
            ROS_ERROR("会话结束失败，错误码: %d.\n", errorCode);
        }
    }
    logout();
    std::cout << "语音合成节点已退出。" << std::endl;
}

void TextToSpeechNode::logout() {
    int errorCode = MSPLogout();
    if (errorCode != MSP_SUCCESS) {
        std::cerr << "合成音频账号注销失败，错误码：" << errorCode << std::endl;
    }
}

void TextToSpeechNode::run() {
    ros::spin();
}

void TextToSpeechNode::whenReceivedTextToSpeechRequest(StringMessagePointer request) {
    if (isThreadEnded) {
        processThread.join();
        isThreadEnded = false;
    }

    if (!processThread.joinable()) {
        std::string content{request->data};
        processThread = std::thread([this, content]() {
            try {
                onlineTextToSpeech(content);
                ROS_INFO("语音合成成功，内容是：%s", content.c_str());
                StatusAndDescriptionMessage result;
                result.status = StatusAndDescriptionMessage::done;
                result.description = content;
                textToSpeechResultPublisher.publish<StatusAndDescriptionMessage>(result);
            } catch (std::runtime_error const& e) {
                ROS_ERROR("语音合成失败，原因是：%s", e.what());
                StatusAndDescriptionMessage result;
                result.status = StatusAndDescriptionMessage::failed;
                result.description = +e.what();
                textToSpeechResultPublisher.publish<StatusAndDescriptionMessage>(result);
            }
            isThreadEnded = true;
        });
    } else {
        StatusAndDescriptionMessage result;
        result.status = StatusAndDescriptionMessage::cancelled;
        result.description = "上一个任务仍在进行中。";
        textToSpeechResultPublisher.publish<StatusAndDescriptionMessage>(result);
        ROS_ERROR("%s", result.description.c_str());
    }
}

void TextToSpeechNode::onlineTextToSpeech(std::string text) {
    Audio::WavePcmHeader wavePcmHeader{};

    int errorCode{MSP_SUCCESS};
    sessionId = QTTSSessionBegin(configs.params.c_str(), &errorCode);
    if (errorCode != MSP_SUCCESS) {
        throw std::runtime_error("会话开始失败");
    }

    errorCode = QTTSTextPut(sessionId.c_str(), text.c_str(), text.length(), NULL);
    if (errorCode != MSP_SUCCESS) {
        throw std::runtime_error("上传文本失败");
    }
    File audioFile(configs.audioFilePath.c_str(), "wb");
    audioFile.write(&wavePcmHeader, sizeof(wavePcmHeader));

    unsigned int audioDataSize{};
    int synthStatus{MSP_TTS_FLAG_STILL_HAVE_DATA};
    do {
        void const* data{QTTSAudioGet(sessionId.c_str(), &audioDataSize, &synthStatus, &errorCode)};
        if (data) {
            audioFile.write(data, audioDataSize);
            wavePcmHeader.subChunk2Size += audioDataSize;
        } else
            std::this_thread::sleep_for(std::chrono::milliseconds(150));
    } while (synthStatus != MSP_TTS_FLAG_DATA_END && errorCode == MSP_SUCCESS);

    if (errorCode != MSP_SUCCESS) {
        throw std::runtime_error("下载音频失败");
    } else {
        wavePcmHeader.chunkSize = wavePcmHeader.subChunk2Size + 36;
    }

    audioFile.seek(0, SEEK_SET);
    audioFile.write(&wavePcmHeader, sizeof(wavePcmHeader.chunkSize));

    errorCode = QTTSSessionEnd(sessionId.c_str(), "Normal");
    if (errorCode != MSP_SUCCESS) {
        throw std::runtime_error("会话结束失败");
    } else {
        sessionId.clear();
    }
    soundplayClient.playWave(configs.audioFilePath);
    auto duration{wavePcmHeader.subChunk2Size / wavePcmHeader.byteRate};
    std::this_thread::sleep_for(std::chrono::seconds(duration + 1));
}
}  // namespace Nodes
}  // namespace xiaohu_robot