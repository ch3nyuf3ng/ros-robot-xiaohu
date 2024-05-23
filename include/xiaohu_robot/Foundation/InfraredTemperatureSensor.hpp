#pragma once

#include "xiaohu_robot/Foundation/Typedefs.hpp"
#ifndef XIAOHU_ROBOT_FOUNDATION_INFRARED_TEMPERATURE_SENSOR_HPP
#define XIAOHU_ROBOT_FOUNDATION_INFRARED_TEMPERATURE_SENSOR_HPP

#include "xiaohu_robot/Foundation/Filesystem.hpp"
#include <cstdint>
#include <vector>

namespace xiaohu_robot {
inline namespace Foundation {
class InfraredTemperatureSensor {
public:
    struct Data {
        Temperature bodyTemperature;
        Temperature ambientTemperature;
    };

    InfraredTemperatureSensor(std::uint8_t i2cAddress = 0x5A);
    ~InfraredTemperatureSensor() = default;

    Data measureTemperature();

private:
    enum RawDataByte {
        ReadMode = 0,
        ObjectTemperatureHigh = 1,
        ObjectTemperatureLow = 2,
        AmbientTemperatureHigh = 3,
        AmbientTemperatureLow = 4
    };
    static double constexpr rawDataScaleFactor{10.0};
    static std::uint8_t constexpr setDataCommand{0x01};
    static std::uint8_t constexpr setModeCommand{0x02};
    static std::uint8_t constexpr readBodyTemperatureMode{0x02};
    std::uint8_t i2cAddress;
    PosixFile deviceFile;

    void findI2cDevice();
    void setMeasurementMode();
    void writeCommands(std::initializer_list<std::uint8_t> commands);
    void readData(std::vector<std::uint8_t>& buffer);
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif