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

using chip_id_t = uint32_t;
static constexpr uint32_t MAX_DMA_BYTES = 4 * 1024 * 1024;

struct dma_buffer {
    void* user_address = nullptr;
    uint64_t physical_address = 0;
    uint64_t size = 0;

    uint64_t get_physical_address() const;
    uint64_t get_user_address() const;
};

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
    uint8_t get_pci_bus() const { return pci_bus; }
    int32_t get_device_fd() { return fd; }
    int32_t get_config_space_fd();
    const dma_buffer& get_dma_transfer_buffer() const { return dma_transfer_buffer; }
    const dma_buffer& get_dma_completion_flag_buffer() const { return dma_completion_flag_buffer; }
    void* get_bar0_uc() { return bar0_uc; }
    size_t get_bar0_uc_offset() { return bar0_uc_offset; }

    template <class T>
    volatile T* get_register_address(uint32_t register_offset) {
        return reinterpret_cast<volatile T*>(get_register_byte_address(register_offset));
    }
    void* get_reg_mapping_and_adjust_offset(uint32_t& address);

    bool reset_by_ioctl();

    static std::unique_ptr<kmd> open(uint32_t index);
    static std::vector<chip_id_t> scan();

   private:
    volatile uint8_t* get_register_byte_address(uint32_t register_offset);

    uint32_t index;
    file_resource fd;
    file_resource config_fd;
    std::filesystem::path sys_bus_path;

    mapping_resource bar0_wc;
    mapping_resource bar0_uc;
    size_t bar0_uc_offset = 0;
    mapping_resource system_reg_mapping;

    uint32_t system_reg_start_offset = 0;   // Registers >= this are system regs, use the mapping.
    uint32_t system_reg_offset_adjust = 0;  // This is the offset of the first reg in the system reg mapping.

    // Device info
    tenstorrent_get_device_info_out device_info;
    uint16_t pci_domain = 0;
    uint8_t pci_bus = 0;
    uint8_t pci_device = 0;
    uint8_t pci_function = 0;
    architecture arch = architecture::invalid;

    // TODO: Initialize!!! Just copied definitions from TTDeviceBase
    // DMA data
    uint32_t next_dma_buf = 0;
	dma_buffer dma_completion_flag_buffer;  // When DMA completes, it writes to this buffer
	dma_buffer dma_transfer_buffer;         // Buffer for large DMA transfers
    uint32_t max_dma_buf_size_log2;
    std::vector<dma_buffer> dma_buffer_mappings;
};

}  // namespace tt::umd
