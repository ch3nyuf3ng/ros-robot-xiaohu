#include "xiaohu_robot/Foundation/InfraredTemperatureSensor.hpp"
#include "serial/serial.h"
#include "xiaohu_robot/Foundation/Debug.hpp"
#include "xiaohu_robot/Foundation/Exceptions.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <algorithm>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <dirent.h>
#include <fcntl.h>
#include <initializer_list>
#include <iostream>
#include <iterator>
#include <linux/i2c-dev.h>
#include <numeric>
#include <stdexcept>
#include <sys/ioctl.h>
#include <thread>
#include <utility>
#include <vector>

namespace xiaohu_robot {
inline namespace Foundation {
std::array<std::uint8_t, 2> const InfraredTemperatureSensorUart::PacketHeader{0xAA, 0xA5};
std::array<std::uint8_t, 1> const InfraredTemperatureSensorUart::PacketTail{0x55};

InfraredTemperatureSensorUart::InfraredTemperatureSensorUart(std::string const& devicePath, std::uint32_t baudrate):
    serialDevice(
        devicePath,
        baudrate,
        serial::Timeout::simpleTimeout(1000),
        serial::eightbits,
        serial::parity_none,
        serial::stopbits_one,
        serial::flowcontrol_none
    ) {
    if (!serialDevice.isOpen()) {
        printMessageThenThrowRuntimeError("无法打开串口设备");
    }

    sendCommand(TemperatureTargetSettingCommand, {BodyTemperature});
    std::vector<std::uint8_t> responseArguments{readCommandResponseArguments(TemperatureTargetSettingCommand)};
    if (responseArguments.front() != 0) {
        serialDevice.close();
        printMessageThenThrowRuntimeError("无法设置测温模式为测体温模式。可能设备不处于 UART 通讯模式中。");
    }
    std::cout << "UART 测温传感器实例已构造。" << std::endl;
}

InfraredTemperatureSensorUart::~InfraredTemperatureSensorUart() {
    if (serialDevice.isOpen()) {
        serialDevice.close();
    }
    std::cout << "UART 测温传感器实例已析构。" << std::endl;
}

InfraredTemperatureSensor::Data InfraredTemperatureSensorUart::measureTemperature() {
    sendCommand(TemperatureMeasurementCommand, {});
    std::vector<std::uint8_t> responseArguments{readCommandResponseArguments(TemperatureMeasurementCommand)};
    if (responseArguments.size() != 5) {
        printContainer("Response args: ", responseArguments);
        printMessageThenThrowRuntimeError("测温响应参数长度不匹配。");
    }
    if (responseArguments[TemperatureTarget] != BodyTemperature) {
        printContainer("Response args: ", responseArguments);
        printMessageThenThrowRuntimeError("测温目标类型不匹配。");
    }
    Temperature bodyTemperature{
        static_cast<double>(
            (responseArguments[TargetTemperatureHighEightBit] << 8 | responseArguments[TargetTemperatureLowEightBit])
            / rawDataScaleFactor
        ),
        UnitTemperature::celcius
    };
    Temperature ambientTemperature{
        static_cast<double>(
            (responseArguments[AmbientTemperatureHighEightBit] << 8 | responseArguments[AmbientTemperatureLowEightBit])
            / rawDataScaleFactor
        ),
        UnitTemperature::celcius
    };
    return {std::move(bodyTemperature), std::move(ambientTemperature)};
}

void InfraredTemperatureSensorUart::sendCommand(std::uint8_t command, std::initializer_list<std::uint8_t> arguments) {
    serialDevice.flush();
    std::vector<std::uint8_t> requestPacket(PacketHeader.begin(), PacketHeader.end());
    std::uint8_t requestPacketLength{static_cast<std::uint8_t>(
        (sizeof(requestPacketLength) + sizeof(command) + arguments.size() + sizeof(std::uint8_t)) / sizeof(std::uint8_t)
    )};
    std::uint8_t requestChecksum{static_cast<std::uint8_t>(
        requestPacketLength + command + std::accumulate(arguments.begin(), arguments.end(), 0)
    )};
    requestPacket.reserve(PacketHeader.size() + requestPacketLength + PacketTail.size());
    requestPacket.push_back(requestPacketLength);
    requestPacket.push_back(command);
    requestPacket.insert(requestPacket.end(), arguments.begin(), arguments.end());
    requestPacket.push_back(requestChecksum);
    requestPacket.insert(requestPacket.end(), std::begin(PacketTail), std::end(PacketTail));
    serialDevice.write(requestPacket);
    // printContainer("Request: ", requestPacket);
}

std::vector<std::uint8_t> InfraredTemperatureSensorUart::readCommandResponseArguments(std::uint8_t command) {
    serialDevice.waitReadable();
    std::vector<std::uint8_t> buffer;
    do {
        serialDevice.read(buffer);
    } while (buffer.back() == 0);
    buffer.erase(buffer.begin(), buffer.end() - 1);
    serialDevice.read(buffer, PacketHeader.size() - 1);
    if (!std::equal(std::begin(PacketHeader), std::end(PacketHeader), buffer.begin())) {
        std::vector<std::uint8_t> actualResponseHeader(buffer.begin(), buffer.begin() + PacketHeader.size());
        printContainer("实际响应数据包包头: ", actualResponseHeader);
        printContainer("期待响应数据包包头: ", PacketHeader);
        printMessageThenThrowRuntimeError("响应数据包包头不匹配。");
    }
    buffer.clear();
    serialDevice.read(buffer, 1);
    std::uint8_t responsePacketLength{buffer.front()};
    buffer.clear();
    buffer.reserve(responsePacketLength + PacketTail.size() - 1);
    serialDevice.read(buffer, responsePacketLength + PacketTail.size() - 1);
    std::uint8_t responseCommand{buffer.front()};
    if (responseCommand != command) {
        printContainer("缓存：", buffer);
        std::cerr << "实际响应指令：";
        printElement(responseCommand);
        std::cerr << std::endl;
        std::cerr << "预期响应指令：";
        printElement(command);
        std::cerr << std::endl;
        printMessageThenThrowRuntimeError("响应指令不匹配。");
    }
    std::vector<std::uint8_t> responseArguments(buffer.begin() + 1, buffer.end() - PacketTail.size() - 1);
    std::uint8_t responseChecksum{buffer[1 + responseArguments.size()]};
    std::uint8_t expectedResponseChecksum{static_cast<uint8_t>(
        responsePacketLength + responseCommand + std::accumulate(responseArguments.begin(), responseArguments.end(), 0)
    )};
    if (responseChecksum != expectedResponseChecksum) {
        printContainer("缓存：", buffer);
        std::cerr << "实际校验码：";
        printElement(responseChecksum);
        std::cerr << std::endl;
        std::cerr << "预期校验码：";
        printElement(expectedResponseChecksum);
        std::cerr << std::endl;
        printContainer("Response: ", buffer);
        printMessageThenThrowRuntimeError("响应数据包校验码检验失败。");
    }
    std::vector<std::uint8_t> responseTail(buffer.end() - PacketTail.size(), buffer.end());
    if (!std::equal(PacketTail.begin(), PacketTail.end(), responseTail.begin())) {
        printContainer("缓存：", buffer);
        std::vector<std::uint8_t> actualResponseTail(buffer.end() - PacketTail.size(), buffer.end());
        printContainer("实际响应数据包包头: ", actualResponseTail);
        printContainer("期待响应数据包包头: ", PacketHeader);
        printMessageThenThrowRuntimeError("响应数据包包尾不匹配。");
    }
    return responseArguments;
}

InfraredTemperatureSensorI2c::InfraredTemperatureSensorI2c(std::uint8_t i2cAddress, std::string const& devicePath):
    i2cAddress{i2cAddress},
    deviceFile(devicePath.c_str(), O_RDWR) {
    if (devicePath.empty()) {
        findI2cDevice();
    }
    setMeasurementMode();
}

InfraredTemperatureSensorI2c::~InfraredTemperatureSensorI2c() {
    std::cout << "I2C 测温传感器实例已析构。" << std::endl;
}

void InfraredTemperatureSensorI2c::findI2cDevice() {
    Directory directory("/dev/");
    for (auto const& entry : directory) {
        std::string entryName{entry.d_name};
        std::size_t constexpr expectedIndex{0};
        if (entryName.find("i2c-") != expectedIndex) {
            continue;
        }
        std::string fileName{"/dev/" + entryName};
        try {
            deviceFile = PosixFile(fileName.c_str(), O_RDWR);
        } catch (std::runtime_error const& e) {
            continue;
        }
        if (ioctl(deviceFile.getDescriptor(), I2C_SLAVE, i2cAddress) < 0) {
            continue;
        } else {
            return;
        }
    }
    printMessageThenThrowRuntimeError("未找到可用的 i2c 总线。");
}

void InfraredTemperatureSensorI2c::setMeasurementMode() {
    writeCommands({0x02, 0x02});
}

InfraredTemperatureSensorI2c::Data InfraredTemperatureSensorI2c::measureTemperature() {
    writeCommands({0x01});
    std::this_thread::sleep_for(std::chrono::milliseconds(220));

    std::vector<std::uint8_t> rawData(5);
    readData(rawData);
    if (rawData[ReadMode] != readBodyTemperatureMode) {
        printMessageThenThrowRuntimeError("Unexpected measurement type");
    }

    Temperature bodyTemperature{
        static_cast<double>(rawData[ObjectTemperatureHigh] << 8 | rawData[ObjectTemperatureLow]) / rawDataScaleFactor,
        UnitTemperature::celcius
    };
    Temperature ambientTemperature{
        static_cast<double>(rawData[AmbientTemperatureHigh] << 8 | rawData[AmbientTemperatureLow]) / rawDataScaleFactor,
        UnitTemperature::celcius
    };
    return {std::move(bodyTemperature), std::move(ambientTemperature)};
}

void InfraredTemperatureSensorI2c::writeCommands(std::initializer_list<std::uint8_t> commands) {
    if (deviceFile.write(&commands, commands.size()) != commands.size()) {
        printMessageThenThrowRuntimeError("Failed to write to the i2c bus");
    }
}

void InfraredTemperatureSensorI2c::readData(std::vector<std::uint8_t>& buffer) {
    if (deviceFile.read(buffer.data(), buffer.size()) != static_cast<ssize_t>(buffer.size())) {
        printMessageThenThrowRuntimeError("Failed to read from the i2c bus");
    }
}
}  // namespace Foundation
}  // namespace xiaohu_robot