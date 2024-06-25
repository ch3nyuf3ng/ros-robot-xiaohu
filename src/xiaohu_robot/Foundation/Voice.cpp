#include "xiaohu_robot/Foundation/Voice.hpp"

namespace xiaohu_robot {
inline namespace Foundation {
namespace XunfeiApi {
std::string const Grammar::standardHeader{"#BNF+IAT 1.0 UTF-8;\n"};
std::string const Grammar::yesOrNo{
    standardHeader + 
    "!grammar yesorno;\n"
    "!start <pred>;\n"
    "<pred>:是|否;"
};

namespace Params {
std::string toString(TextEncoding textEncoding) {
    switch (textEncoding) {
    case TextEncoding::GB2312:
        return "gb2312";
    case TextEncoding::GBK:
        return "gbk";
    case TextEncoding::BIG5:
        return "big5";
    case TextEncoding::UNICODE:
        return "unicode";
    case TextEncoding::GB18030:
        return "gb18030";
    case TextEncoding::UTF8:
        return "utf8";
    }
    return "";
}

std::string toString(TextType textType) {
    switch (textType) {
    case TextType::NormalText:
        return "text";
    case TextType::CSSML:
        return "cssml";
    }
    return "";
}

std::string OnlineTtsParams::generateOnlineTtsParamsString() {
    std::ostringstream oss;

    oss << "engine_type = cloud, ";
    oss << "speed = " << speed << ", ";
    oss << "volume = " << volume << ", ";
    oss << "pitch = " << pitch << ", ";
    oss << "rdn = " << static_cast<int>(numberPronunciation) << ", ";
    oss << "rcn = " << static_cast<int>(numberOnePronunciation) << ", ";
    oss << "text_encoding = " << toString(textEncoding) << ", ";
    oss << "sample_rate = " << static_cast<int>(sampleRate) << ", ";
    oss << "background_sound = " << withBackgroundSound << ", ";
    oss << "aue = " << audioEncodingAndCompressionLevel << ", ";
    oss << "ttp = " << toString(textType);

    return oss.str();
}
}  // namespace Params
}  // namespace XunfeiApi
}  // namespace Foundation
}  // namespace xiaohu_robot