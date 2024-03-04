// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "device/kmd.h"

#include <fcntl.h>
#include <linux/pci.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <memory>
#include <sstream>

#include "common/logger.hpp"
#include "device/resources.h"
#include "ioctl.h"

namespace tt::umd {

static constexpr uint32_t GS_BAR0_WC_MAPPING_SIZE = (156 << 20) + (10 << 21) + (18 << 24);

static std::filesystem::path get_sys_bus_path(
    uint16_t pci_domain, uint8_t pci_bus, uint8_t pci_device, uint8_t pci_function) {
    std::stringstream ss;

    ss << "/sys/bus/pci/devices/";
    ss << std::setfill('0') << std::setw(4) << std::hex << static_cast<uint32_t>(pci_domain);
    ss << ':' << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint32_t>(pci_bus);
    ss << ':' << std::setfill('0') << std::setw(2) << std::hex << static_cast<uint32_t>(pci_device);
    ss << '.' << std::setfill('0') << std::setw(1) << std::hex << static_cast<uint32_t>(pci_function);
    return ss.str();
}

static int32_t read_int_from_file(const std::filesystem::path& path, bool verify_only_int = true) {
    std::ifstream file(path);
    std::string line;
    if (std::getline(file, line)) {
        std::size_t index;
        int32_t result = std::stoi(line, &index, 0);

        if (verify_only_int && index != line.size()) {
            throw std::runtime_error(
                "Error reading int from file: " + path.string() + ". Expected only an int on first line, but found '" +
                line + "'.");
        }
        return result;
    } else {
        throw std::runtime_error("Error reading int from file: " + path.string());
    }
}

uint64_t dma_buffer::get_physical_address() const {
    log_assert(physical_address, "DMA Buffer not initialized");
    return reinterpret_cast<uint64_t>(physical_address);
}

uint64_t dma_buffer::get_user_address() const {
    log_assert(user_address, "DMA Buffer not initialized");
    return reinterpret_cast<uint64_t>(user_address);
}

kmd::kmd(file_resource fd) : fd(std::move(fd)) {}

kmd::~kmd() {
    // TODO:
    // for (auto &&buf : dma_buffer_mappings) {
    //     munmap(buf.pBuf, buf.size);
    // }

    // if (sysfs_config_fd != -1) {
    //     close(sysfs_config_fd);
    // }
}

std::unique_ptr<kmd> kmd::open(uint32_t index) {
    // Open device
    std::string device_name = "/dev/tenstorrent/" + std::to_string(index);
    file_resource device_fd(::open(device_name.c_str(), O_RDWR | O_CLOEXEC));

    if (!device_fd) {
        throw std::runtime_error(std::string("Failed opening a handle for device ") + std::to_string(index));
    }

    // Get device info
    std::unique_ptr<kmd> kmd{new tt::umd::kmd(std::move(device_fd))};

    tenstorrent_get_device_info device_info = {};
    device_info.in.output_size_bytes = sizeof(device_info.out);

    if (::ioctl(kmd->fd, TENSTORRENT_IOCTL_GET_DEVICE_INFO, &device_info) == -1) {
        throw std::runtime_error(std::string("Get device info failed on device ") + std::to_string(index) + ".");
    }

    kmd->device_info = device_info.out;
    kmd->pci_domain = device_info.out.pci_domain;
    kmd->pci_bus = device_info.out.bus_dev_fn >> 8;
    kmd->pci_device = PCI_SLOT(device_info.out.bus_dev_fn);
    kmd->pci_function = PCI_FUNC(device_info.out.bus_dev_fn);
    kmd->sys_bus_path = get_sys_bus_path(kmd->pci_domain, kmd->pci_bus, kmd->pci_device, kmd->pci_function);

    // Get architecture
    auto device_id = device_info.out.device_id;
    auto revision_id = read_int_from_file(kmd->sys_bus_path / "revision");

    if (device_id == 0xfaca) {
        kmd->arch = architecture::grayskull;
    } else if (device_id == 0x401e) {
        kmd->arch = revision_id == 0x01 ? architecture::wormhole_b0 : architecture::wormhole;
    } else {
        throw std::runtime_error(std::string("Unsupported device ") + std::to_string(index) + ".");
    }

    struct {
        tenstorrent_query_mappings query_mappings;
        tenstorrent_mapping mapping_array[8];
    } mappings = {};
    mappings.query_mappings.in.output_mapping_count =
        sizeof(mappings.mapping_array) / sizeof(mappings.mapping_array[0]);

    if (::ioctl(kmd->fd, TENSTORRENT_IOCTL_QUERY_MAPPINGS, &mappings.query_mappings) == -1) {
        throw std::runtime_error(std::string("Query mappings failed on device ") + std::to_string(index) + ".");
    }

    tenstorrent_mapping bar0_uc_mapping = {}, bar0_wc_mapping = {}, bar2_uc_mapping = {}, bar2_wc_mapping = {};

    for (unsigned int i = 0; i < mappings.query_mappings.in.output_mapping_count; i++) {
        switch (mappings.mapping_array[i].mapping_id) {
            case TENSTORRENT_MAPPING_RESOURCE0_UC: bar0_uc_mapping = mappings.mapping_array[i]; break;
            case TENSTORRENT_MAPPING_RESOURCE0_WC: bar0_wc_mapping = mappings.mapping_array[i]; break;
            case TENSTORRENT_MAPPING_RESOURCE2_UC: bar2_uc_mapping = mappings.mapping_array[i]; break;
            case TENSTORRENT_MAPPING_RESOURCE2_WC: bar2_wc_mapping = mappings.mapping_array[i]; break;
        }
    }

    if (bar0_uc_mapping.mapping_id != TENSTORRENT_MAPPING_RESOURCE0_UC) {
        throw std::runtime_error(std::string("Device ") + std::to_string(index) + " has no BAR0 UC mapping.");
    }

    // Attempt WC mapping first so we can fall back to all-UC if it fails.
    if (bar0_wc_mapping.mapping_id == TENSTORRENT_MAPPING_RESOURCE0_WC) {
        size_t bar0_wc_size = std::min<size_t>(bar0_wc_mapping.mapping_size, GS_BAR0_WC_MAPPING_SIZE);

        kmd->bar0_wc = mapping_resource(
            ::mmap(nullptr, bar0_wc_size, PROT_READ | PROT_WRITE, MAP_SHARED, kmd->fd, bar0_wc_mapping.mapping_base),
            bar0_wc_size);
    }

    size_t bar0_uc_size = 0;

    if (kmd->bar0_wc) {
        // The bottom part of the BAR is mapped WC. Map the top UC.
        bar0_uc_size = bar0_uc_mapping.mapping_size - GS_BAR0_WC_MAPPING_SIZE;
        kmd->bar0_uc_offset = GS_BAR0_WC_MAPPING_SIZE;
    } else {
        // No WC mapping, map the entire BAR UC.
        bar0_uc_size = bar0_uc_mapping.mapping_size;
        kmd->bar0_uc_offset = 0;
    }

    kmd->bar0_uc = mapping_resource(
        mmap(
            nullptr,
            bar0_uc_size,
            PROT_READ | PROT_WRITE,
            MAP_SHARED,
            kmd->fd,
            bar0_uc_mapping.mapping_base + kmd->bar0_uc_offset),
        bar0_uc_size);
    if (!kmd->bar0_uc) {
        throw std::runtime_error(
            std::string("BAR0 UC memory mapping failed for device ") + std::to_string(index) + ".");
    }

    // TODO: Why do we do this?!?
    // if (!kmd->bar0_wc) {
    //     kmd->bar0_wc = kmd->bar0_uc;
    // }

    if (kmd->arch == architecture::wormhole || kmd->arch == architecture::wormhole_b0) {
        if (bar2_uc_mapping.mapping_id != TENSTORRENT_MAPPING_RESOURCE2_UC) {
            throw std::runtime_error(std::string("Device ") + std::to_string(index) + " has no BAR4 UC mapping.");
        }

        kmd->system_reg_mapping = mapping_resource(
            ::mmap(
                nullptr,
                bar2_uc_mapping.mapping_size,
                PROT_READ | PROT_WRITE,
                MAP_SHARED,
                kmd->fd,
                bar2_uc_mapping.mapping_base),
            bar2_uc_mapping.mapping_size);

        if (!kmd->system_reg_mapping) {
            throw std::runtime_error(
                std::string("BAR4 UC memory mapping failed for device ") + std::to_string(index) + ".");
        }

        kmd->system_reg_start_offset = (512 - 16) * 1024 * 1024;
        kmd->system_reg_offset_adjust = (512 - 32) * 1024 * 1024;
    }

    // TODO: Implement

    // auto ttdev = std::make_unique<TTDevice>(TTDevice::open(device_id));
    // PCIdevice device;
    // device.id = device_id;
    // device.hdev = ttdev.get();
    // device.vendor_id = ttdev->device_info.vendor_id;
    // device.device_id = ttdev->device_info.device_id;
    // device.subsystem_vendor_id = ttdev->device_info.subsystem_vendor_id;
    // device.subsystem_id = ttdev->device_info.subsystem_id;
    // device.dwBus = ttdev->pci_bus;
    // device.dwSlot = ttdev->pci_device;
    // device.dwFunction = ttdev->pci_function;
    // device.BAR_addr = read_bar0_base(ttdev.get());
    // device.BAR_size_bytes = ttdev->bar0_uc_size;
    // device.revision_id = get_revision_id(ttdev.get());
    // ttdev.release();
    return kmd;
}

std::vector<chip_id_t> kmd::scan() {
    std::vector<chip_id_t> found_devices;
    std::filesystem::path dev_path = "/dev/tenstorrent";

    for (const auto& entry : std::filesystem::directory_iterator(dev_path)) {
        const std::string& filename = entry.path().filename().string();

        if (filename.empty() || !std::all_of(filename.begin(), filename.end(), ::isdigit)) {
            continue;
        }
        if (!entry.is_character_file()) {
            continue;
        }

        std::istringstream iss(filename);
        chip_id_t chip_id;

        iss >> chip_id;
        found_devices.push_back(chip_id);
    }

    std::sort(found_devices.begin(), found_devices.end());

    return found_devices;
}

int32_t kmd::get_config_space_fd() {
    // TODO: This is not thread safe!!!
    if (!config_fd) {
        std::string path = sys_bus_path / "config";

        config_fd = ::open(path.c_str(), O_RDWR);
        if (!config_fd) {
            config_fd = ::open(path.c_str(), O_RDONLY);
        }
    }

    return config_fd;
}

volatile uint8_t* kmd::get_register_byte_address(uint32_t register_offset) {
    void* reg_mapping;
    if (system_reg_mapping != nullptr && register_offset >= system_reg_start_offset) {
        register_offset -= system_reg_offset_adjust;
        reg_mapping = system_reg_mapping;
    } else if (register_offset < bar0_wc.length()) {
        reg_mapping = bar0_wc;
    } else {
        register_offset -= bar0_uc_offset;
        reg_mapping = bar0_uc;
    }
    return static_cast<uint8_t*>(reg_mapping) + register_offset;
}

void* kmd::get_reg_mapping_and_adjust_offset(uint32_t& address) {
    if (system_reg_mapping != nullptr && address >= system_reg_start_offset) {
        address -= system_reg_offset_adjust;
        return system_reg_mapping;
    } else {
        return bar0_wc ? bar0_wc : bar0_uc;  // TODO: This is cause of TODO in open() function
    }
}

bool kmd::reset_by_ioctl() {
    struct tenstorrent_reset_device reset_device;
    memset(&reset_device, 0, sizeof(reset_device));

    reset_device.in.output_size_bytes = sizeof(reset_device.out);
    reset_device.in.flags = 0;

    if (ioctl(fd, TENSTORRENT_IOCTL_RESET_DEVICE, &reset_device) == -1) {
        return false;
    }

    return (reset_device.out.result == 0);
}

}  // namespace tt::umd
