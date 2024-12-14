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


constexpr size_t FOUR_GIGS = 1ULL << 32;
constexpr size_t TWO_MEGS = 1ULL << 21;

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
    static constexpr uint32_t window_shifts[] = { 21, 37 };
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

class NocWindowPool
{
private:
    // Is doing this in Rust any less aesthetically offensive?  As difficult?
    // Using unique_ptr deleter wrecks the type signature.  I could pass a
    // std::function into NocWindow() to do the deletion in ~NocWindow() but
    // that's less explicit, less efficient... Ugh.
    struct WindowDeleter
    {
        NocWindowPool* pool;
        void operator()(NocWindow* ptr) { if (ptr) pool->release(ptr); }
    };

public:
    using WindowHandle = std::unique_ptr<NocWindow, WindowDeleter>;

    NocWindowPool(NocDriver& driver, size_t window_size, size_t quantity = 10)
    {
        for (size_t i = 0; i < quantity; ++i) {
            auto window = driver.open_window(window_size, {});
            windows.push(std::move(window));
        }
    }

    // Potential optimization: pass in desired configuration and prioritize
    // reusing windows that match the configuration.  Tradeoff between:
    // Current situation: always reconfigure (syscall + reg writes + barrier)
    // Reuse windows: Lookup overhead + potential reconfiguration anyway
    //
    // I think this is fine for now.  If you want to optimize, it makes sense to
    // hold a window for each configuration you expect to use.
    WindowHandle acquire()
    {
        std::unique_lock<std::mutex> lock(mtx);
        cv.wait(lock, [this]() { return !windows.empty(); });

        // Take one down...
        NocWindow* window = windows.front().release();
        windows.pop();

        // Pass it around...
        return WindowHandle(window, WindowDeleter{this});
    }

private:
    void release(NocWindow* window)
    {
        std::unique_lock<std::mutex> lock(mtx);
        windows.push(std::unique_ptr<NocWindow>(window));
        cv.notify_one();
    }

    std::queue<std::unique_ptr<NocWindow>> windows;
    std::mutex mtx;
    std::condition_variable cv;
};

using ManagedNocWindow = NocWindowPool::WindowHandle;

class NOC
{
    NocDriver driver;
    NocWindowPool windows_2M;
    NocWindowPool windows_128G;

public:
    NOC()
        : driver()
        , windows_2M(driver, TWO_MEGS, 10)
        , windows_128G(driver, FOUR_GIGS, 5)
    {
    }

    // I don't know if the window pool thing really makes sense... I am doing
    // this because the type signature of what get_window returns is stupid,
    // and I don't want to have to deal with it up in the UMD.
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

    auto get_window(size_t window_size)
    {
        if (window_size == TWO_MEGS) {
            return windows_2M.acquire();
        }

        if (window_size == FOUR_GIGS) {
            return windows_128G.acquire();
        }

        throw std::runtime_error("Invalid window size");
    }

    auto get_window(size_t window_size, noc_window_config config)
    {
        auto window = get_window(window_size);
        window->reconfigure(config);
        return window;
    }

    uint32_t read32(uint32_t x, uint32_t y, uint64_t addr)
    {
        uint32_t val;
        read(x, y, addr, &val, sizeof(val));
        return val;
    }

    void write32(uint32_t x, uint32_t y, uint64_t addr, uint32_t val)
    {
        write(x, y, addr, &val, sizeof(val));
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

        noc_window_config config{
            .addr = window_addr,
            .x_end = x,
            .y_end = y,
        };

        auto window = get_window(window_size, config);
        void* src = window->data() + offset;
        std::memcpy(dst, src, size);
    }

    void write(uint32_t x, uint32_t y, uint64_t addr, const void* src, size_t size)
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

        noc_window_config config{
            .addr = window_addr,
            .x_end = x,
            .y_end = y,
        };

        auto window = get_window(window_size, config);
        void* dst = window->data() + offset;
        std::memcpy(dst, src, size);
    }
};
