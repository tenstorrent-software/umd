/* SPDX-FileCopyrightText: © 2024 Tenstorrent Inc.
 * SPDX-License-Identifier: GPL-2.0-only
 */

#pragma once

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "x280_ioctl.h"

#include <cassert>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <queue>
#include <stdexcept>

#include "logger_.hpp"


constexpr size_t FOUR_GIGS = 1ULL << 32;
constexpr size_t TWO_MEGS = 1ULL << 21;

namespace tt::umd {

class NocWindow
{
    enum Kind { Size2M, Size128G };

    int fd;
    Kind kind;
    noc_window_handle handle{};
    size_t mapped_size;
    uint8_t* window{nullptr};

public:
    NocWindow(int fd, uint64_t size, noc_window_config config)
        : fd(fd)
        , kind(size <= (1ULL << 21) ? Kind::Size2M : Kind::Size128G)
    {
        if (ioctl(fd, ioctl_alloc[kind], &handle) < 0) {
            throw std::runtime_error("Couldn't allocate window");
        }

        if (size > handle.mmap_size) {
            ioctl(fd, ioctl_dealloc[kind], &handle);
            throw std::runtime_error("Size too large for window");
        }

        config.window_id = handle.window_id;
        if (ioctl(fd, ioctl_config[kind], &config) < 0) {
            ioctl(fd, ioctl_dealloc[kind], &handle);
            throw std::runtime_error("Couldn't configure window");
        }

        void* mem = mmap(0, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, handle.mmap_offset);
        if (mem == MAP_FAILED) {
            ioctl(fd, ioctl_dealloc[kind], &handle);
            throw std::runtime_error("Couldn't map window");
        }

        mapped_size = size;
        window = static_cast<uint8_t*>(mem);
    }

    void reconfigure(noc_window_config config)
    {
        config.window_id = handle.window_id;
        if (ioctl(fd, ioctl_config[kind], &config) < 0) {
            throw std::runtime_error("Couldn't reconfigure window");
        }
    }

    size_t size() const { return mapped_size; }
    uint8_t* data() const { return window; }

    ~NocWindow()
    {
        if (window) {
            munmap(window, mapped_size);
        }
        ioctl(fd, ioctl_dealloc[kind], &handle);
    }

private:
    static constexpr uint32_t ioctl_alloc[] = { L2CPU_IOCTL_ALLOC_2M, L2CPU_IOCTL_ALLOC_128G };
    static constexpr uint32_t ioctl_dealloc[] = { L2CPU_IOCTL_DEALLOC_2M, L2CPU_IOCTL_DEALLOC_128G };
    static constexpr uint32_t ioctl_config[] = { L2CPU_IOCTL_CONFIG_2M, L2CPU_IOCTL_CONFIG_128G };
};

class NocDriver
{
    int fd;

public:
    NocDriver()
        : fd(open("/dev/l2cpu-noc", O_RDWR))
    {
        if (fd < 0) {
            throw std::runtime_error("Couldn't open driver");
        }
    }

    std::unique_ptr<NocWindow> open_window(uint64_t size, noc_window_config config)
    {
        return std::make_unique<NocWindow>(fd, size, config);
    }

    ~NocDriver()
    {
        if (fd >= 0) {
            close(fd);
        }
    }
};

class NOC
{
    NocDriver driver;

public:
    NOC()
        : driver()
    {
    }

    std::unique_ptr<NocWindow> allocate_window(size_t size, noc_window_config config)
    {
        if (size == TWO_MEGS) {
            return driver.open_window(size, config);
        }

        if (size == FOUR_GIGS) {
            return driver.open_window(size, config);
        }

        throw std::runtime_error("Invalid window size");
    }

    uint32_t read32(uint32_t x, uint32_t y, uint64_t addr)
    {
        uint32_t value;
        read(x, y, addr, &value, sizeof(value));
        return value;
    }

    void write32(uint32_t x, uint32_t y, uint64_t addr, uint32_t val)
    {
        write_block(x, y, addr, &val, sizeof(val));
    }

    void read(uint32_t x, uint32_t y, uint64_t addr, void* dst, size_t size)
    {
        size_t window_size = size > TWO_MEGS ? FOUR_GIGS : TWO_MEGS;
        uint64_t window_addr = addr & ~(window_size - 1);
        uint64_t offset = addr & (window_size - 1);

        if (size > FOUR_GIGS) {
            throw std::runtime_error("Read size too large");
        }

        if (size + offset > window_size) {
            throw std::runtime_error("Read crosses window boundary");
        }

        if (size <= 8 && (addr & (size - 1))) {
            throw std::runtime_error("Unaligned read");
        }

        noc_window_config config{
            .addr = window_addr,
            .x_end = x,
            .y_end = y,
        };

        auto window = allocate_window(window_size, config);
        void* src = window->data() + offset;

        switch (size) {
        case 1:
            UMD_INFO("Reading 1 byte from {},{} at {:#x}", x, y, addr);
            *reinterpret_cast<uint8_t*>(dst) = *reinterpret_cast<volatile uint8_t*>(src);
            return;
        case 2:
            UMD_INFO("Reading 2 bytes from {},{} at {:#x}", x, y, addr);
            *reinterpret_cast<uint16_t*>(dst) = *reinterpret_cast<volatile uint16_t*>(src);
            return;
        case 4:
            UMD_INFO("Reading 4 bytes from {},{} at {:#x}", x, y, addr);
            *reinterpret_cast<uint32_t*>(dst) = *reinterpret_cast<volatile uint32_t*>(src);
            return;
        case 8:
            UMD_INFO("Reading 8 bytes from {},{} at {:#x}", x, y, addr);
            *reinterpret_cast<uint64_t*>(dst) = *reinterpret_cast<volatile uint64_t*>(src);
            return;
        }

        // for (size_t i = 0; i < size; ++i) {
        //     reinterpret_cast<uint8_t*>(dst)[i] = reinterpret_cast<volatile uint8_t*>(src)[i];
        // }
        UMD_INFO("Reading {} bytess from {},{} at {:#x}", size, x, y, addr);
        memcpy(dst, src, size);
    }

    void write_block(uint32_t x, uint32_t y, uint64_t addr, const void* src, size_t size)
    {
        size_t window_size = size > TWO_MEGS ? FOUR_GIGS : TWO_MEGS;
        uint64_t window_addr = addr & ~(window_size - 1);
        uint64_t offset = addr & (window_size - 1);

        if (size > FOUR_GIGS) {
            throw std::runtime_error("Read size too large");
        }

        if (size + offset > window_size) {
            throw std::runtime_error("Read crosses window boundary");
        }

        if (size <= 8 && (addr & (size - 1))) {
            throw std::runtime_error("Unaligned write");
        }

        noc_window_config config{
            .addr = window_addr,
            .x_end = x,
            .y_end = y,
        };

        auto window = allocate_window(window_size, config);
        void* dst = window->data() + offset;

        UMD_INFO("About to write to {},{} at {:#x}", x, y, addr);
        UMD_INFO("Window address: {:#x}", window_addr);
        UMD_INFO("Offset: {:#x}", offset);

        switch (size) {
        case 1:
            *reinterpret_cast<volatile uint8_t*>(dst) = *reinterpret_cast<const uint8_t*>(src);
            UMD_INFO("Writing 1 byte to {},{} at {:#x}", x, y, addr);
            return;
        case 2:
            *reinterpret_cast<volatile uint16_t*>(dst) = *reinterpret_cast<const uint16_t*>(src);
            UMD_INFO("Writing 2 bytes to {},{} at {:#x}", x, y, addr);
            return;
        case 4:
            *reinterpret_cast<volatile uint32_t*>(dst) = *reinterpret_cast<const uint32_t*>(src);
            UMD_INFO("Writing 4 bytes to {},{} at {:#x}", x, y, addr);
            return;
        case 8:
            *reinterpret_cast<volatile uint64_t*>(dst) = *reinterpret_cast<const uint64_t*>(src);
            UMD_INFO("Writing 8 bytes to {},{} at {:#x}", x, y, addr);
            return;
        }

        UMD_INFO("Writing {} bytess to {},{} at {:#x}", size, x, y, addr);
        memcpy(dst, src, size);
    }
};

} // namespace tt::umd