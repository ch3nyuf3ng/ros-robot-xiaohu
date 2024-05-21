#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_FILEWRAPPER_HPP
#define XIAOHU_ROBOT_FOUNDATION_FILEWRAPPER_HPP

#include <cstdio>

namespace xiaohu_robot {
inline namespace Foundation {
class File {
public:
    File(const char* filename, const char* mode);
    ~File();

    File(const File&) = delete;
    File& operator=(const File&) = delete;

    File(File&& other) noexcept;
    File& operator=(File&& other) noexcept;

    void close();
    FILE* get() const;

    int seek(long offset, int whence);
    std::size_t write(const void* datePtr, std::size_t dataSize, std::size_t dataCount = 1);

private:
    FILE* filePtr;
};
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif  // FILEWRAPPER_H
