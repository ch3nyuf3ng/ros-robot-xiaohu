#include "xiaohu_robot/Foundation/Filesystem.hpp"
#include <cstring>
#include <fcntl.h>
#include <stdexcept>
#include <string>
#include <unistd.h>
#include <utility>

using namespace std::literals;

namespace xiaohu_robot {
inline namespace Foundation {
inline namespace Filesystem {
BufferedFile::BufferedFile(char const* filename, char const* mode):
    filePointer{std::fopen(filename, mode)} {
    if (!filePointer) {
        throw std::runtime_error("Failed to open file: "s + filename);
    }
}

BufferedFile::~BufferedFile() {
    close();
}

BufferedFile::BufferedFile(BufferedFile&& other) noexcept:
    filePointer(other.filePointer) {
    other.filePointer = nullptr;
}

BufferedFile& BufferedFile::operator=(BufferedFile&& other) noexcept {
    if (this != &other) {
        this->close();
        std::swap(filePointer, other.filePointer);
    }
    return *this;
}

void BufferedFile::close() {
    if (filePointer) {
        std::fclose(filePointer);
        filePointer = nullptr;
    }
}

FILE* BufferedFile::get() const {
    return filePointer;
}

int BufferedFile::seek(long offset, int whence) {
    return fseek(filePointer, offset, whence);
}

std::size_t BufferedFile::write(void const* datePtr, std::size_t dataSize, std::size_t dataCount) {
    return fwrite(datePtr, dataSize, dataCount, filePointer);
}

PosixFile::PosixFile(char const* filename, int flags):
    fileDescriptor{::open(filename, flags)} {
    if (fileDescriptor < 0) {
        throw std::runtime_error("Failed to open file: " + std::string(filename) + " (" + std::strerror(errno) + ")");
    }
}

PosixFile::~PosixFile() {
    close();
}

PosixFile::PosixFile(PosixFile&& other) noexcept:
    fileDescriptor(other.fileDescriptor) {
    other.fileDescriptor = -1;
}

PosixFile& PosixFile::operator=(PosixFile&& other) noexcept {
    if (this != &other) {
        close();
        std::swap(fileDescriptor, other.fileDescriptor);
    }
    return *this;
}

int PosixFile::getDescriptor() const {
    return fileDescriptor;
}

ssize_t PosixFile::read(void* buffer, std::size_t count) {
    return ::read(fileDescriptor, buffer, count);
}

ssize_t PosixFile::write(void const* buffer, std::size_t count) {
    return ::write(fileDescriptor, buffer, count);
}

void PosixFile::close() {
    if (fileDescriptor >= 0) {
        ::close(fileDescriptor);
        fileDescriptor = -1;
    }
}

Directory::Directory(std::string const& path):
    directoryPointer{opendir(path.c_str())} {
    if (!directoryPointer) {
        throw std::runtime_error("Failed to open directory: " + path);
    }
}

Directory::~Directory() {
    close();
}

Directory::Directory(Directory&& other) noexcept:
    directoryPointer{other.directoryPointer} {
    other.directoryPointer = nullptr;
}

Directory& Directory::operator=(Directory&& other) noexcept {
    if (this != &other) {
        close();
        std::swap(directoryPointer, other.directoryPointer);
    }
    return *this;
}

void Directory::close() {
    if (directoryPointer) {
        closedir(directoryPointer);
        directoryPointer = nullptr;
    }
}

Directory::Iterator::Iterator(DIR* directoryPointer, dirent* entryPointer):
    directoryPointer{directoryPointer},
    entryPointer{entryPointer} {}

Directory::Iterator& Directory::Iterator::operator++() {
    entryPointer = readdir(directoryPointer);
    return *this;
}

const dirent& Directory::Iterator::operator*() const {
    return *entryPointer;
}

const dirent* Directory::Iterator::operator->() const {
    return entryPointer;
}

bool Directory::Iterator::operator!=(Iterator const& other) const {
    return entryPointer != other.entryPointer;
}

Directory::Iterator Directory::begin() {
    return Iterator{directoryPointer, readdir(directoryPointer)};
}

Directory::Iterator Directory::end() {
    return Iterator{directoryPointer, nullptr};
}
}  // namespace Filesystem
}  // namespace Foundation
}  // namespace xiaohu_robot

