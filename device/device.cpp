// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "device/device.h"

#include <memory>

#include "device/architecture_implementation.h"
#include "device/kmd.h"

namespace tt::umd {

device::device(architecture arch, chip_id_t logical_device_id) : arch(arch), logical_device_id(logical_device_id) {
    arch_impl = architecture_implementation::create(arch);
}

pci_device::pci_device(std::unique_ptr<tt::umd::kmd> kmd, chip_id_t logical_device_id) :
    device(kmd->get_architecture(), logical_device_id), kmd(std::move(kmd)) {}

std::shared_ptr<device> pci_device::open(std::unique_ptr<tt::umd::kmd> kmd, chip_id_t logical_device_id) {
    std::shared_ptr<device> device{new pci_device(std::move(kmd), logical_device_id)};

    // TODO: Implement
    return std::move(device);
}

void pci_device::read_from_device(void* memory, size_t size, cxy_pair core, uint64_t address) {
    // TODO: Implement
    // if (fallback_tlb == "REG_TLB") {
    //     read_mmio_device_register(mem_ptr, core, addr, size, fallback_tlb);
    // } else {
    //     read_device_memory(mem_ptr, core, addr, size, fallback_tlb);
    // }
}

}  // namespace tt::umd
