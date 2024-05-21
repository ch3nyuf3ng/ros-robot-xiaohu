#include "xiaohu_robot/Foundation/Audio.hpp"
#include <alsa/pcm.h>
#include <cassert>
#include <chrono>
#include <cstddef>
#include <exception>
#include <iostream>
#include <mutex>
#include <ostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

namespace xiaohu_robot {
inline namespace Foundation {
inline namespace Audio {
Recorder::Recorder(DataCallback whenPeriodDataIsReady, EndCallback whenStopped, WAVEFORMATEX waveFormat):
    recordingThread{[this]() {
        try {
            record();
        } catch (...) {
            std::lock_guard<std::mutex> lock(exceptionPointerMutex);
            exceptionPointer = std::current_exception();
        }
    }},
    whenPeriodDataIsReady{std::move(whenPeriodDataIsReady)},
    whenStopped{std::move(whenStopped)},
    waveFormat{std::move(waveFormat)} {
    try {
        openDefaultInputDevice();
        setHardwareParams();
        setSoftwareParams();
        state = State::Ready;
        std::cout << "录音机已构造。" << std::endl;
    } catch (const std::runtime_error& e) {
        std::cerr << "录音机构造失败。" << e.what() << std::endl;
        close();
        throw;
    }
}

Recorder::~Recorder() {
    close();
    std::cout << "录音机已析构。" << std::endl;
}

void Recorder::close() noexcept {
    state = State::Destructing;
    if (recordingThread.joinable()) {
        recordingThread.join();
    }
    if (waveInputHandle) {
        snd_pcm_close(waveInputHandle);
        waveInputHandle = nullptr;
    }
}

void Recorder::start() {
    checkThreadsException();
    if (state != State::Ready) {
        return;
    }
    if (snd_pcm_start(waveInputHandle) != NoError) {
        throw std::runtime_error("录音机启动失败。");
    }
    state = State::Recording;
}

void Recorder::stop() {
    checkThreadsException();
    if (state != State::Recording) {
        return;
    }
    state = State::Stopping;
    {
        std::lock_guard<std::mutex> lock(waveInputHandleMutex);
        if (snd_pcm_drop(waveInputHandle) != NoError) {
            throw std::runtime_error("录音机停止失败。");
        }
    }
    whenStopped();
    state = State::Ready;
    std::cout << "录音机已停止。" << std::endl;
}

void Recorder::checkThreadsException() {
    std::lock_guard<std::mutex> lock(exceptionPointerMutex);
    if (exceptionPointer) {
        state = State::Destructing;
        std::rethrow_exception(exceptionPointer);
    }
}

void Recorder::openDefaultInputDevice() {
    if (snd_pcm_open(&waveInputHandle, "default", SND_PCM_STREAM_CAPTURE, 0) != NoError) {
        throw std::runtime_error("Cannot get default PCM device.");
    }
}

void Recorder::setHardwareParams() {
    snd_pcm_hw_params_t* hardwareParams;
    snd_pcm_hw_params_alloca(&hardwareParams);
    std::lock_guard<std::mutex> lock(waveInputHandleMutex);
    if (snd_pcm_hw_params_any(waveInputHandle, hardwareParams) < NoError) {
        throw std::runtime_error("Broken configuration for this PCM.");
    }
    if (snd_pcm_hw_params_set_access(waveInputHandle, hardwareParams, SND_PCM_ACCESS_RW_INTERLEAVED) < NoError) {
        throw std::runtime_error("Access type not available.");
    }
    snd_pcm_format_t alsaFormat = snd_pcm_build_linear_format(
        waveFormat.wBitsPerSample, waveFormat.wBitsPerSample, waveFormat.wBitsPerSample == 8 ? 1 : 0, 0
    );
    if (alsaFormat == SND_PCM_FORMAT_UNKNOWN) {
        throw std::runtime_error("Invalid Wave format.");
    }
    if (snd_pcm_hw_params_set_format(waveInputHandle, hardwareParams, alsaFormat) < NoError) {
        throw std::runtime_error("Sample format not available.");
    }
    if (snd_pcm_hw_params_set_channels(waveInputHandle, hardwareParams, waveFormat.nChannels) < NoError) {
        throw std::runtime_error("Channels count not available.");
    }
    unsigned int actualSampleRate{waveFormat.nSamplesPerSec};
    if (snd_pcm_hw_params_set_rate_near(waveInputHandle, hardwareParams, &actualSampleRate, nullptr) < NoError) {
        throw std::runtime_error("Set sample rate failed.");
    }
    if (actualSampleRate != waveFormat.nSamplesPerSec) {
        throw std::runtime_error("Sample rate mismatch.");
    }
    if (snd_pcm_hw_params_set_period_time_near(waveInputHandle, hardwareParams, &periodTime, nullptr) < NoError) {
        throw std::runtime_error("Set period time failed.");
    }
    if (snd_pcm_hw_params_set_buffer_time_near(waveInputHandle, hardwareParams, &bufferTime, nullptr) < NoError) {
        throw std::runtime_error("Set buffer time failed.");
    }
    if (snd_pcm_hw_params_get_period_size(hardwareParams, &periodFrames, nullptr) < NoError) {
        throw std::runtime_error("Get period size failed.");
    }
    if (snd_pcm_hw_params_get_buffer_size(hardwareParams, &bufferFrames) < NoError) {
        throw std::runtime_error("Get buffer size failed.");
    } else if (bufferFrames == periodFrames) {
        throw std::runtime_error("Buffer frames shouldn't be equal to period frames.");
    }
    bitsPerframe = waveFormat.wBitsPerSample;
    if (snd_pcm_hw_params(waveInputHandle, hardwareParams) < NoError) {
        throw std::runtime_error("Install hardware parameters failed.");
    }
}

void Recorder::setSoftwareParams() {
    snd_pcm_sw_params_t* softwareParams;
    snd_pcm_sw_params_alloca(&softwareParams);
    std::lock_guard<std::mutex> lock(waveInputHandleMutex);
    if (snd_pcm_sw_params_current(waveInputHandle, softwareParams) < NoError) {
        throw std::runtime_error("Get current software params failed.");
    }
    if (snd_pcm_sw_params_set_avail_min(waveInputHandle, softwareParams, periodFrames) < NoError) {
        throw std::runtime_error("Set available minimum avail frames failed.");
    }
    if (snd_pcm_sw_params_set_start_threshold(waveInputHandle, softwareParams, bufferFrames * 2) < NoError) {
        throw std::runtime_error("Ser start threshold failed.");
    }
    if (snd_pcm_sw_params(waveInputHandle, softwareParams) < NoError) {
        throw std::runtime_error("Unable to install software params.");
    }
}

void Recorder::record() {
    std::cout << "录音线程开始。" << std::endl;
    while (true) {
        checkThreadsException();
        if (state == State::Constructing || state == State::Ready || state == State::Stopping) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            continue;
        } else if (state == State::Destructing) {
            std::cout << "录音线程结束。" << std::endl;
            break;
        }
        std::thread userTaskThread{[this, buffer = readPeriodData()]() {
            try {
                whenPeriodDataIsReady(buffer);
            } catch (...) {
                std::lock_guard<std::mutex> lock(exceptionPointerMutex);
                exceptionPointer = std::current_exception();
            }
        }};
        userTaskThread.detach();
    }
}

std::vector<char> Recorder::readPeriodData() {
    std::lock_guard<std::mutex> lock(waveInputHandleMutex);
    std::size_t unreadFrames{periodFrames};
    std::vector<char> buffer(periodFrames * bitsPerframe / 8);
    char* dataPointer{buffer.data()};
    while (unreadFrames > 0) {
        snd_pcm_sframes_t result{snd_pcm_readi(waveInputHandle, dataPointer, unreadFrames)};
        if (result >= 0 && static_cast<size_t>(result) < unreadFrames) {
            snd_pcm_wait(waveInputHandle, 100);
        } else if (result < 0) {
            xrunRecovery(result);
        }
        unreadFrames -= result;
        dataPointer += unreadFrames * bitsPerframe / 8;
    }
    return buffer;
}

void Recorder::xrunRecovery(int errorCode) {
    if (errorCode == -EPIPE && snd_pcm_prepare(waveInputHandle) != NoError) {
        throw std::runtime_error("Can't recover from overrun, prepare failed.");
    } else if (errorCode == -ESTRPIPE) {
        while ((errorCode = snd_pcm_resume(waveInputHandle)) == -EAGAIN) {
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
        }
        if (errorCode != NoError && snd_pcm_prepare(waveInputHandle) != NoError) {
            throw std::runtime_error("Can't recover from overrun, prepare failed.");
        }
    } else {
        throw std::runtime_error("Unknown PCM read error.");
    }
}
}  // namespace Audio
}  // namespace Foundation
}  // namespace xiaohu_robot