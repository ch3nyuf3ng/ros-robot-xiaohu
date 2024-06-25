#pragma once

#ifndef XIAOHU_ROBOT_FOUNDATION_FILEWRAPPER_HPP
#define XIAOHU_ROBOT_FOUNDATION_FILEWRAPPER_HPP

#include <cstdio>
#include <dirent.h>
#include <string>

namespace xiaohu_robot {
inline namespace Foundation {
inline namespace Filesystem {
class BufferedFile final {
public:
    BufferedFile(char const* filename, char const* mode);
    ~BufferedFile();

    BufferedFile(const BufferedFile&) = delete;
    BufferedFile& operator=(const BufferedFile&) = delete;

    BufferedFile(BufferedFile&& other) noexcept;
    BufferedFile& operator=(BufferedFile&& other) noexcept;

    void close();
    FILE* get() const;

    int seek(long offset, int whence);
    std::size_t write(const void* datePtr, std::size_t dataSize, std::size_t dataCount = 1);

private:
    FILE* filePointer;
};

class PosixFile final {
public:
    PosixFile(char const* filename, int flags);
    ~PosixFile();

    PosixFile(const PosixFile&) = delete;
    PosixFile& operator=(const PosixFile&) = delete;

    PosixFile(PosixFile&& other) noexcept;
    PosixFile& operator=(PosixFile&& other) noexcept;

    int getDescriptor() const;

    ssize_t read(void* buffer, std::size_t count);
    ssize_t write(const void* buffer, std::size_t count);

private:
    int fileDescriptor;

    void close();
};

class Directory final {
public:
    explicit Directory(std::string const& path);
    ~Directory();

    Directory(Directory const&) = delete;
    Directory& operator=(Directory const&) = delete;

    Directory(Directory&& other) noexcept;
    Directory& operator=(Directory&& other) noexcept;

    class Iterator final {
    public:
        using iterator_category = std::input_iterator_tag;
        using value_type = dirent;
        using difference_type = std::ptrdiff_t;
        using pointer = dirent*;
        using reference = dirent&;

        Iterator(DIR* directoryPointer, dirent* entryPointer);
        Iterator& operator++();
        const dirent& operator*() const;
        const dirent* operator->() const;
        bool operator!=(const Iterator& other) const;

    private:
        DIR* directoryPointer;
        dirent* entryPointer;
    };

    Iterator begin();
    Iterator end();

private:
    DIR* directoryPointer;

    void close();
};
}  // namespace Filesystem
}  // namespace Foundation
}  // namespace xiaohu_robot

#endif
