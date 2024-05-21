#include "xiaohu_robot/Foundation/File.hpp"
#include <stdexcept>
#include <string>
#include <utility>

namespace xiaohu_robot {
inline namespace Foundation {
File::File(const char* filename, const char* mode):
    filePtr{std::fopen(filename, mode)} {
    if (!filePtr) {
        throw std::runtime_error("Failed to open file: " + std::string(filename));
    }
}

File::~File() {
    close();
}

File::File(File&& other) noexcept:
    filePtr(other.filePtr) {
    other.filePtr = nullptr;
}

File& File::operator=(File&& other) noexcept {
    if (this != &other) {
        this->close();
        std::swap(filePtr, other.filePtr);
    }
    return *this;
}

void File::close() {
    if (filePtr) {
        std::fclose(filePtr);
        filePtr = nullptr;
    }
}

FILE* File::get() const {
    return filePtr;
}

int File::seek(long offset, int whence) {
    return fseek(filePtr, offset, whence);
}

// 实现 fwrite 封装
std::size_t File::write(const void* datePtr, std::size_t dataSize, std::size_t dataCount) {
    return fwrite(datePtr, dataSize, dataCount, filePtr);
}
}  // namespace Foundation
}  // namespace xiaohu_robot

