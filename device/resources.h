/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <sys/mman.h>
#include <unistd.h>

#include <cstdint>

namespace tt::umd {

class file_resource {
    file_resource(const file_resource&) = delete;
    file_resource& operator=(const file_resource&) = delete;

   public:
    file_resource() = default;
    file_resource(int32_t fd) : fd(fd) {}
    ~file_resource() {
        if (fd != -1) {
            ::close(fd);
        }
    }
    file_resource(file_resource&& other) noexcept : fd(other.fd) { other.fd = -1; }
    file_resource& operator=(file_resource&& other) noexcept {
        if (this != &other) {
            if (fd != -1) {
                ::close(fd);
            }
            fd = other.fd;
            other.fd = -1;
        }
        return *this;
    }

    operator int32_t() const { return fd; }
    operator bool() const { return fd != -1; }
    bool operator!() const { return fd == -1; }

   private:
    int32_t fd = -1;
};

class mapping_resource {
    mapping_resource(const mapping_resource&) = delete;
    mapping_resource& operator=(const mapping_resource&) = delete;

   public:
    mapping_resource() = default;
    mapping_resource(void* memory, size_t size) : memory(memory), size(size) {}
    ~mapping_resource() { release(); }
    mapping_resource(mapping_resource&& other) noexcept : memory(other.memory), size(other.size) {
        other.memory = nullptr;
        other.size = 0;
    }
    mapping_resource& operator=(mapping_resource&& other) noexcept {
        if (this != &other) {
            release();
            memory = other.memory;
            size = other.size;
            other.memory = nullptr;
            other.size = 0;
        }
        return *this;
    }

    operator const void*() const { return memory; }
    operator void*() { return memory; }
    operator bool() const { return memory != nullptr && memory != MAP_FAILED && size != 0; }
    bool operator!() const { return memory == nullptr || memory == MAP_FAILED || size == 0; }

    void* ptr() { return memory; }
    const void* ptr() const { return memory; }
    size_t length() const { return size; }

   private:
    void release() {
        if (memory != nullptr && memory != MAP_FAILED) {
            ::munmap(memory, size);
        }
    }

    void* memory = nullptr;
    size_t size = 0;
};

}  // namespace tt::umd
