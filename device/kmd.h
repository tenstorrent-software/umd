/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

#include "device/architecture.h"

#include "device/ioctl.h"
#include "device/resources.h"

namespace tt::umd {

using chip_id_t = int32_t;

class kmd {
   private:
    kmd(const kmd&) = delete;
    kmd(kmd&&) = delete;
    kmd& operator=(const kmd&) = delete;
    kmd& operator=(kmd&&) = delete;

    kmd(file_resource fd);

   public:
    ~kmd();

    architecture get_architecture() const { return arch; }

    uint32_t get_index() const { return index; }
    int32_t get_device_fd() { return fd; }
    int32_t get_config_space_fd();

    static std::unique_ptr<kmd> open(uint32_t index);
    static std::vector<chip_id_t> scan();

   private:
    uint32_t index;
    file_resource fd;
    file_resource config_fd;
    std::filesystem::path sys_bus_path;

    mapping_resource bar0_wc;
    mapping_resource bar0_uc;
    std::size_t bar0_uc_offset = 0;
    mapping_resource system_reg_mapping;

    std::uint32_t system_reg_start_offset = 0;  // Registers >= this are system regs, use the mapping.
    std::uint32_t system_reg_offset_adjust = 0; // This is the offset of the first reg in the system reg mapping.

    // Device info
    tenstorrent_get_device_info_out device_info;
    uint16_t pci_domain = 0;
    uint8_t pci_bus = 0;
    uint8_t pci_device = 0;
    uint8_t pci_function = 0;
    architecture arch = architecture::invalid;
};

}  // namespace tt::umd
