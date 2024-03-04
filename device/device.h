/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <functional>
#include <memory>
#include <unordered_map>

#include "device/architecture.h"
#include "device/architecture_implementation.h"
#include "device/kmd.h"
#include "device/tlb.h"
#include "device/xy_pair.h"

namespace boost::interprocess{
    class named_mutex;
}

namespace tt::umd {

class device {
   private:
    device(const device&) = delete;
    device(device&&) = delete;
    device& operator=(const device&) = delete;
    device& operator=(device&&) = delete;

   public:
    virtual ~device() = default;

    const architecture_implementation* get_architecture_implementation() const { return arch_impl.get(); }
    architecture get_architecture() const { return arch; }

    // Returns logical device id. Usually represents index in device_collection.
    chip_id_t get_logical_device_id() const { return logical_device_id; }

    virtual bool is_mmio_capable() const = 0;

    virtual void read(void* memory, uint32_t size_in_bytes, xy_pair core, uint64_t address, const std::string& fallback_tlb) = 0;

   protected:
    device(architecture arch, chip_id_t logical_device_id);

    chip_id_t logical_device_id;
    architecture arch;
    std::unique_ptr<architecture_implementation> arch_impl;
};

// Represents mmio capable device
class pci_device : public device {
   private:
    struct dynamic_tlb_configuration {
        uint32_t index;
        std::shared_ptr<boost::interprocess::named_mutex> mutex;
    };

   public:
    using map_core_to_tlb_func = std::function<int32_t(xy_pair)>;

    static std::shared_ptr<device> open(std::unique_ptr<tt::umd::kmd> kmd, chip_id_t logical_device_id);

    bool is_mmio_capable() const override { return true; }

    void setup_core_to_tlb_map(map_core_to_tlb_func mapping_function) { map_core_to_tlb = std::move(mapping_function); }
    void set_dma_buffer_size(uint32_t size) { dma_buffer_size = size; }
    void configure_tlb(
        xy_pair core, uint32_t tlb_index, uint32_t address, tlb_data_ordering ordering = tlb_data_ordering::relaxed);

    void read(void* memory, uint32_t size_in_bytes, xy_pair core, uint64_t address, const std::string& fallback_tlb) override;

   private:
    pci_device(std::unique_ptr<tt::umd::kmd> kmd, chip_id_t logical_device_id);

    bool reset_by_sysfs();
    bool auto_reset_board();
    bool is_hardware_hung();
    void detect_ffffffff_read(uint32_t data_read = 0xffffffffu);
    void memcpy_from_device(void *dest, const void *src, std::size_t num_bytes);
    void write_regs(uint32_t byte_address, uint32_t word_len, const void *data);
    void write_tlb_reg(uint32_t byte_address, uint64_t value);
    uint32_t pcie_dma_transfer_turbo(
        uint32_t chip_address, uint32_t host_physical_address, uint32_t size_bytes, bool write);
    void read_block(uint32_t byte_address, uint32_t num_bytes, uint8_t* buffer_address);
    bool address_in_tlb_space(uint32_t address, uint32_t size_in_bytes, int32_t tlb_index, uint32_t tlb_size);
    dynamic_tlb set_dynamic_tlb(
        uint32_t tlb_index, xy_pair start, xy_pair end, uint32_t address, bool multicast, tlb_data_ordering ordering);
    dynamic_tlb set_dynamic_tlb(
        uint32_t tlb_index, xy_pair target, uint32_t address, tlb_data_ordering ordering = tlb_data_ordering::relaxed) {
        return set_dynamic_tlb(tlb_index, xy_pair(0, 0), target, address, false, ordering);
    }

    void read_memory(void* memory, size_t size_in_bytes, xy_pair core, uint64_t address, const std::string& fallback_tlb);
    void read_registers(void* memory, size_t size_in_bytes, xy_pair core, uint64_t address, const std::string& fallback_tlb);

    std::unique_ptr<tt::umd::kmd> kmd;
    map_core_to_tlb_func map_core_to_tlb;
    std::unordered_map<int32_t, int32_t> tlb_config_map;
    uint32_t dma_buffer_size = 0;  // The setting should not exceed MAX_DMA_BYTES
    std::unordered_map<xy_pair, xy_pair> harvested_coord_translation;
    std::unordered_map<std::string, int32_t> dynamic_tlb_config;

    // TODO: Used to be global variables in the original code
    bool read_checking_enabled;
    bool use_msi_for_dma = false;  // Whether to wait for MSI after DMA transfer, or poll a variable
    uint32_t dma_block_size_read_threshold_bytes =
        0;  // 0 - never use DMA. Otherwise use DMA for all blocks larger than this size
    uint32_t dma_block_size_write_threshold_bytes =
        0;  // 0 - never use DMA. Otherwise use DMA for all blocks larger than this size
    uint32_t csm_pcie_ctrl_dma_request_offset = 0;  // Address in CSM where the DMA request structure resides
    uint32_t dma_trigger_address = 0;               // Address where the trigger for transfer resides
    uint32_t arc_misc_cntl_address = 0;             // To trigger arc interrupt
};

// Represents non-mmio capable device
// class network_device : public device {
//     // TODO: Implement
// };

}  // namespace tt::umd
