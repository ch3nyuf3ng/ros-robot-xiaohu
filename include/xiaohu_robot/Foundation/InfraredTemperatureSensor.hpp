#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_INFRARED_TEMPERATURE_SENSOR_HPP
#define XIAOHU_ROBOT_FOUNDATION_INFRARED_TEMPERATURE_SENSOR_HPP

#include "serial/serial.h"
#include "xiaohu_robot/Foundation/Filesystem.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <array>
#include <cstdint>
#include <initializer_list>
#include <vector>

namespace xiaohu_robot {
inline namespace Foundation {
class InfraredTemperatureSensor {
public:
    struct Data {
        Temperature bodyTemperature;
        Temperature ambientTemperature;
    };

    virtual ~InfraredTemperatureSensor() = default;
    virtual Data measureTemperature() = 0;
};

class InfraredTemperatureSensorUart final: public InfraredTemperatureSensor {
public:
    InfraredTemperatureSensorUart(
        std::string const& devicePath = "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0",
        std::uint32_t baudrate = 38400
    );
    ~InfraredTemperatureSensorUart();

    Data measureTemperature() override;

private:
    static double constexpr rawDataScaleFactor{10.0};
    static std::array<std::uint8_t, 2> const PacketHeader;
    static std::array<std::uint8_t, 1> const PacketTail;

    enum Commands {
        TemperatureMeasurementCommand = 0x01,
        TemperatureTargetSettingCommand = 0x02,
        LowTemperatureBlackBodyMarkingCommand = 0x03,
        HighTemperatureBlackBodyMarkingCommand = 0x04,
        CommunicationProtocolSettingCommand = 0x05
    };

    enum TemperatureTargetArgument {
        ObjectTemperature = 0x01,
        BodyTemperature = 0x02
    };

    enum TemperaturMeasurementResponseArguments {
        TemperatureTarget = 0,
        TargetTemperatureHighEightBit = 1,
        TargetTemperatureLowEightBit = 2,
        AmbientTemperatureHighEightBit = 3,
        AmbientTemperatureLowEightBit = 4
    };

    serial::Serial serialDevice;

    void sendCommand(std::uint8_t command, std::initializer_list<std::uint8_t> arguments);
    std::vector<std::uint8_t> readCommandResponseArguments(std::uint8_t command);
};

class InfraredTemperatureSensorI2c: public InfraredTemperatureSensor {
public:
    InfraredTemperatureSensorI2c(
        std::uint8_t i2cAddress = 0x5A,
        std::string const& devicePath = ""
    );
    ~InfraredTemperatureSensorI2c();

    Data measureTemperature() override;

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