#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_WAV_PCM_HEADER_HPP
#define XIAOHU_ROBOT_FOUNDATION_WAV_PCM_HEADER_HPP

#include <alsa/asoundlib.h>
#include <atomic>
#include <cstdint>
#include <exception>
#include <functional>
#include <mutex>
#include <thread>
#include <vector>

namespace xiaohu_robot {
inline namespace Foundation {
inline namespace Audio {
struct WavePcmHeader final {
    static int constexpr pcmAudioFormat{1};
    static int constexpr pcmSubChunk1Size{16};
    static int constexpr defaultSampleRate{16000};
    static int constexpr defaultBitsPerSample{16};
    static int constexpr defaultNumChannels{1};

    std::uint8_t chunkId[4]{'R', 'I', 'F', 'F'};
    std::uint32_t chunkSize{36};
    std::uint8_t format[4]{'W', 'A', 'V', 'E'};
    std::uint8_t subChunk1Id[4]{'f', 'm', 't', ' '};
    std::uint32_t subChunk1Size{pcmSubChunk1Size};
    std::uint16_t audioFormat{pcmAudioFormat};
    std::uint16_t numChannels{defaultNumChannels};
    std::uint32_t sampleRate{defaultSampleRate};
    std::uint32_t byteRate{defaultSampleRate * defaultNumChannels * defaultBitsPerSample / 8};
    std::uint16_t blockAlign{defaultNumChannels * defaultBitsPerSample / 8};
    std::uint16_t bitsPerSample{defaultBitsPerSample};
    std::uint8_t subChunk2Id[4]{'d', 'a', 't', 'a'};
    std::uint32_t subChunk2Size{0};

    void setDataSize(std::int32_t size) {
        subChunk2Size = size;
        chunkSize = 36 + size;
    }
};

struct WAVEFORMATEX final {
    static int constexpr pcmAudioFormat{1};
    static int constexpr defaultNumChannels{1};
    static int constexpr defaultBitsPerSample{16};
    static int constexpr defaultSampleRate{16000};

    std::uint16_t wFormatTag{pcmAudioFormat};
    std::uint16_t nChannels{defaultNumChannels};
    std::uint32_t nSamplesPerSec{defaultSampleRate};
    std::uint32_t nAvgBytesPerSec{defaultSampleRate * defaultNumChannels * defaultBitsPerSample / 8};
    std::uint16_t nBlockAlign{defaultNumChannels * defaultBitsPerSample / 8};
    std::uint16_t wBitsPerSample{defaultBitsPerSample};
    std::uint16_t cbSize{0};
};

class Recorder {
public:
    using DataCallback = std::function<void(std::vector<char> const& data)>;
    using EndCallback = std::function<void()>;
    enum class State {
        Constructing,
        Ready,
        Recording,
        Stopping,
        Destructing
    };

    Recorder(DataCallback whenPeriodDataIsReady, EndCallback whenStopped, WAVEFORMATEX waveFormat);
    Recorder() = delete;
    Recorder(Recorder const&) = delete;
    Recorder(Recorder&&) = delete;
    Recorder& operator=(Recorder const&) = delete;
    Recorder& operator=(Recorder&&) = delete;
    ~Recorder();

    void start();
    void stop();
    void checkThreadsException();

private:
    using WaveInputHandle = snd_pcm_t*;
    static int constexpr NoError{0};
    static unsigned int const defaultBufferTime{500'000};
    static unsigned int const defaultPeriodTime{defaultBufferTime / 4};

    std::thread recordingThread;
    std::atomic<State> state{State::Constructing};
    std::exception_ptr exceptionPointer;
    std::mutex exceptionPointerMutex;
    WaveInputHandle waveInputHandle{nullptr};
    std::mutex waveInputHandleMutex;
    unsigned int periodTime{defaultPeriodTime};
    unsigned int bufferTime{defaultBufferTime};
    unsigned long periodFrames{0};
    unsigned long bufferFrames{0};
    unsigned int bitsPerframe{0};

    DataCallback whenPeriodDataIsReady;
    EndCallback whenStopped;
    WAVEFORMATEX waveFormat;

    void close() noexcept;
    void openDefaultInputDevice();
    void setHardwareParams();
    void setSoftwareParams();
    void record();
    std::vector<char> readPeriodData();
    void xrunRecovery(int errorCode);
};
}  // namespace Audio
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif