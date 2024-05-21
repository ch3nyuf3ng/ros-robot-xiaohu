#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_VOICE_HPP
#define XIAOHU_ROBOT_FOUNDATION_VOICE_HPP

#include "xiaohu_robot/Foundation/CommonConfigs.hpp"
#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
namespace XunfeiApi {
namespace Params {
enum class EngineType {
    Cloud,
    Local
};

enum class VoiceName {
    MandarinXiaoYan,
    MandarinXuJiu,
    MandarinXiaoPing,
    MandarinXiaoJing,
    MandarinXuXiaobao
};

enum class NumberPronunciation {
    PreferNumber = 0,
    NumberOnly = 1,
    StringOnly = 2,
    PreferString = 3
};

enum class NumberOnePronunciation {
    PronunceYao = 0,
    PronunceYi = 1
};

enum class TextEncoding {
    GB2312,
    GBK,
    BIG5,
    UNICODE,
    GB18030,
    UTF8
};

enum class SampleRate {
    High,
    Low
};

enum class TextType {
    NormalText,
    CSSML
};

std::string toString(TextEncoding textEncoding);
std::string toString(TextType textType);

struct OnlineTtsParams final {
    EngineType engineType{EngineType::Cloud};
    VoiceName voiceName{VoiceName::MandarinXiaoYan};
    int speed{50};
    int volume{50};
    int pitch{50};
    NumberPronunciation numberPronunciation{NumberPronunciation::PreferNumber};
    NumberOnePronunciation numberOnePronunciation{NumberOnePronunciation::PronunceYi};
    TextEncoding textEncoding{TextEncoding::UTF8};
    SampleRate sampleRate{SampleRate::High};
    bool withBackgroundSound{false};
    std::string audioEncodingAndCompressionLevel{"speex-wb;7"};
    TextType textType{TextType::NormalText};

    std::string generateOnlineTtsParamsString();
};

struct LocalAsrParams final {
    int const asrThreshold{50};
    std::string const asrResourcePath{CommonConfigs::packagePath() + "/lib/common.jet"};
    std::string const asrGrammarBuildPath{CommonConfigs::packagePath() + "/lib/grammar/"};
};
}  // namespace Params

struct Grammar final {
    static std::string const yesOrNo;

    Grammar() = delete;
    Grammar(Grammar const&) = delete;
    Grammar(Grammar&&) = delete;
    Grammar& operator=(Grammar const&) = delete;
    Grammar& operator=(Grammar&&) = delete;

private:
    static std::string const standardHeader;
};
}  // namespace XunfeiApi
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif