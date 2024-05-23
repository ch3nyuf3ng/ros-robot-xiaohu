#include "xiaohu_robot/Foundation/InfraredTemperatureSensor.hpp"
#include "xiaohu_robot/Foundation/Measurement.hpp"
#include "xiaohu_robot/Foundation/Typedefs.hpp"
#include <dirent.h>
#include <fcntl.h>
#include <initializer_list>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <sys/ioctl.h>
#include <thread>

namespace xiaohu_robot {
inline namespace Foundation {
InfraredTemperatureSensor::InfraredTemperatureSensor(std::uint8_t i2cAddress):
    i2cAddress{i2cAddress},
    deviceFile("", O_RDONLY) {
    findI2cDevice();
    setMeasurementMode();
}

void InfraredTemperatureSensor::findI2cDevice() {
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
    throw std::runtime_error("No available I2C bus found.");
}

void InfraredTemperatureSensor::setMeasurementMode() {
    writeCommands({0x02, 0x02});
}

InfraredTemperatureSensor::Data InfraredTemperatureSensor::measureTemperature() {
    writeCommands({0x01});
    std::this_thread::sleep_for(std::chrono::milliseconds(220));

    std::vector<std::uint8_t> rawData(5);
    readData(rawData);
    if (rawData[ReadMode] != readBodyTemperatureMode) {
        throw std::runtime_error("Unexpected measurement type");
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

void InfraredTemperatureSensor::writeCommands(std::initializer_list<std::uint8_t> commands) {
    if (deviceFile.write(&commands, commands.size()) != commands.size()) {
        throw std::runtime_error("Failed to write to the i2c bus");
    }
}

void InfraredTemperatureSensor::readData(std::vector<std::uint8_t>& buffer) {
    if (deviceFile.read(buffer.data(), buffer.size()) != static_cast<ssize_t>(buffer.size())) {
        throw std::runtime_error("Failed to read from the i2c bus");
    }
}
}  // namespace Foundation
}  // namespace xiaohu_robot