#pragma once

#ifndef XIAOHU_ROBOT_NODES_SPEECH_RECOGNITION_NODE
#define XIAOHU_ROBOT_NODES_SPEECH_RECOGNITION_NODE

#include "xiaohu_robot/Foundation/Audio.hpp"
#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include "xiaohu_robot/Foundation/NodeControl.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include "xiaohu_robot/Foundation/Voice.hpp"
#include <atomic>
#include <chrono>
#include <thread>

namespace xiaohu_robot {
inline namespace Nodes {
class SpeechRecognitionNode final: public Runnable {
public:
    struct Configs final {
        std::string speechRecognitionRequestTopic{CommonConfigs::speechRecognitionRequestTopic};
        std::string speechRecognotionResultTopic{CommonConfigs::speechRecognitionResultTopic};
        std::string xfyunApiLoginParams{CommonConfigs::xfyunApiLoginParams};
        std::string grammar{XunfeiApi::Grammar::yesOrNo};
        XunfeiApi::Params::LocalAsrParams asrParams{};
        NodeBasicConfigs nodeBasicConfigs{};
    };

    SpeechRecognitionNode(Configs);
    SpeechRecognitionNode(SpeechRecognitionNode const&) = delete;
    SpeechRecognitionNode(SpeechRecognitionNode&&) noexcept = delete;
    SpeechRecognitionNode& operator=(SpeechRecognitionNode const&) = delete;
    SpeechRecognitionNode& operator=(SpeechRecognitionNode&&) noexcept = delete;
    ~SpeechRecognitionNode();
    void run() override;

private:
    struct GrammarHandle final {
        std::atomic_bool hasBuilt{false};
        std::string grammarId{};
    };

    struct RecorderContext final {
        bool isFirstBlock{true};
        std::atomic_bool hasGotResult{false};
        std::string result{};
    };

    NodeHandle nodeHandle;
    Subscriber speechRecognitionRequestSubsriber;
    Publisher speechRecognitionResultPublisher;
    SoundplayClient soundplayClient;

    std::atomic_bool isThreadEnded{false};
    std::thread processThread{};
    std::string sessionId{};

    GrammarHandle grammarHandle{};
    std::string asrSessionParams{};

    Configs configs;

    void logout();
    void endSession();
    void uploadGrammar();
    void whenReceivedSpeechRecognitionRequest(StringMessage::ConstPtr const& request);
    std::string speechRecognitionFromMicrophone(std::chrono::seconds duration);
    Recorder::DataCallback whenPeriodDataIsReady(RecorderContext& context);
    Recorder::EndCallback whenRecorderStopped(RecorderContext& context);
    static int whenGrammarHasBuilt(int errorCode, char const* grammarId, void* userData);
};
}  // namespace Nodes
}  // namespace xiaohu_robot

#endif