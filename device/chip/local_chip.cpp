/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "umd/device/chip/local_chip.h"

#include "logger.hpp"
#include "umd/device/tt_device/tlb_manager.h"
#include "umd/device/tt_device/tt_device.h"
#include "umd/device/types/blackhole_eth.h"

namespace tt::umd {

LocalChip::LocalChip(tt_SocDescriptor soc_descriptor, int pci_device_id) :
    Chip(soc_descriptor), tt_device_(TTDevice::create(pci_device_id)) {
    initialize_tlb_manager();
}

void LocalChip::initialize_tlb_manager() {
    auto tlb_manager = tt_device_->get_tlb_manager();
    // Setup default dynamic tlbs.
    tlb_manager->set_dynamic_tlb_config(
        "LARGE_READ_TLB", tt_device_->get_architecture_implementation()->get_mem_large_read_tlb());
    tlb_manager->set_dynamic_tlb_config(
        "LARGE_WRITE_TLB", tt_device_->get_architecture_implementation()->get_mem_large_write_tlb());
    tlb_manager->set_dynamic_tlb_config("REG_TLB", tt_device_->get_architecture_implementation()->get_reg_tlb());
    tlb_manager->set_dynamic_tlb_config(
        "SMALL_READ_WRITE_TLB", tt_device_->get_architecture_implementation()->get_small_read_write_tlb());
}

LocalChip::LocalChip(std::unique_ptr<TTDevice> tt_device, const ChipInfo chip_info) :
    Chip(
        tt_SocDescriptor(
            tt_SocDescriptor::get_soc_descriptor_path(
                tt_device->get_arch(), chip_info.board_type, chip_info.chip_uid.asic_location),
            chip_info.noc_translation_enabled,
            chip_info.harvesting_masks),
        chip_info),
    tt_device_(std::move(tt_device)) {
    initialize_tlb_manager();
}

TTDevice* LocalChip::get_tt_device() { return tt_device_.get(); }

bool LocalChip::is_mmio_capable() const { return true; }

void LocalChip::wait_eth_cores_training(const uint32_t timeout_per_core) {
    log_assert(
        get_tt_device()->get_arch() == tt::ARCH::BLACKHOLE,
        "Waiting for training of ETH cores is supported only for Blackhole LocalChip.");

    const std::vector<CoreCoord> eth_cores = get_soc_descriptor().get_cores(CoreType::ETH);
    for (const CoreCoord& eth_core : eth_cores) {
        TTDevice* tt_device = get_tt_device();

        const tt_xy_pair eth_core_pair = {eth_core.x, eth_core.y};

        uint32_t postcode;
        auto start = std::chrono::system_clock::now();
        while (true) {
            tt_device->read_from_device(&postcode, eth_core_pair, blackhole::BOOT_RESULTS_ADDR, sizeof(postcode));

            if (postcode == blackhole::POSTCODE_ETH_INIT_PASS) {
                return;
            }

            if (postcode == blackhole::POSTCODE_ETH_INIT_FAIL) {
                // TODO: Exception should be thrown here. ETH connections are very flaky
                // on Blackhole right now. When this is fixed we can throw the exception here.
                // Since we are not going to do any remote IO at the moment it is fine to just log the error.
                log_error("Eth core ({}, {}) failed to initialize", eth_core_pair.x, eth_core_pair.y);
                return;
            }

            auto end = std::chrono::system_clock::now();  // End time
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
            if (duration.count() > timeout_per_core) {
                // TODO: Exception should be thrown here. ETH connections are very flaky
                // on Blackhole right now. When this is fixed we can throw the exception here.
                // Since we are not going to do any remote IO at the moment it is fine to just log the error.
                log_error(
                    "Timed out after waiting {} seconds for eth core ({}, {}) to initialize",
                    timeout_per_core,
                    eth_core_pair.x,
                    eth_core_pair.y);
                return;
            }
        }
    }
}

}  // namespace tt::umd
