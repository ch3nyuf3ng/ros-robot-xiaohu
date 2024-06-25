#pragma once

#ifndef XIAOHU_ROBOT_TEXT_TO_SPEACH_NODE_HPP
#define XIAOHU_ROBOT_TEXT_TO_SPEACH_NODE_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/Voice.hpp"
#include <atomic>
#include <thread>

namespace xiaohu_robot {
inline namespace Nodes {
class TextToSpeechNode final: public Runnable {
public:
    struct Configs final {
        std::string textToSpeechRequestTopic{CommonConfigs::TextToSpeechRequestTopic};
        std::string textToSpeechResultTopic{CommonConfigs::TextToSpeechResultTopic};
        std::string loginParams{CommonConfigs::XfyunApiLoginParams};
        std::string params{XunfeiApi::Params::OnlineTtsParams{}.generateOnlineTtsParamsString()};
        std::string audioFilePath = "/dev/shm/tts_result.wav";
        NodeBasicConfigs nodeBasicConfig{};
    };

    TextToSpeechNode(Configs);
    TextToSpeechNode(TextToSpeechNode const&) = delete;
    TextToSpeechNode& operator=(TextToSpeechNode const&) = delete;
    TextToSpeechNode(TextToSpeechNode&&) = delete;
    TextToSpeechNode& operator=(TextToSpeechNode&&) = delete;
    ~TextToSpeechNode();

    void run() override;

private:
    NodeHandle nodeHandle{};

    Subscriber textToSpeechRequestSubscriber;
    Publisher textToSpeechResultPublisher;

    SoundplayClient soundplayClient{};

    Configs configs;

    std::atomic_bool isThreadEnded{false};
    std::thread processThread{};
    std::string sessionId{};

    void logout();
    void whenReceivedTextToSpeechRequest(StringMessage::ConstPtr const& request);
    void onlineTextToSpeech(std::string text);
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif