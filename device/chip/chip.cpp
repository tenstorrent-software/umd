/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "umd/device/chip/chip.h"

#include "umd/device/architecture_implementation.h"

namespace tt::umd {

Chip::Chip(tt_SocDescriptor soc_descriptor) : soc_descriptor_(soc_descriptor) {}

tt_SocDescriptor& Chip::get_soc_descriptor() { return soc_descriptor_; }

TTDevice* Chip::get_tt_device() { return nullptr; }

void Chip::set_default_params(ARCH arch) {
    auto architecture_implementation = tt::umd::architecture_implementation::create(arch);

    // Default initialize l1_address_params based on detected arch
    l1_address_params = architecture_implementation->get_l1_address_params();

    // Default initialize dram_address_params.
    dram_address_params = {0u};

    // Default initialize host_address_params based on detected arch
    host_address_params = architecture_implementation->get_host_address_params();

    // Default initialize eth_interface_params based on detected arch
    eth_interface_params = architecture_implementation->get_eth_interface_params();

    // Default initialize noc_params based on detected arch
    noc_params = architecture_implementation->get_noc_params();
}

void Chip::set_barrier_address_params(const barrier_address_params& barrier_address_params_) {
    l1_address_params.tensix_l1_barrier_base = barrier_address_params_.tensix_l1_barrier_base;
    l1_address_params.eth_l1_barrier_base = barrier_address_params_.eth_l1_barrier_base;
    dram_address_params.DRAM_BARRIER_BASE = barrier_address_params_.dram_barrier_base;
}

}  // namespace tt::umd
