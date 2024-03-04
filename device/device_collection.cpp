// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "device/device_collection.h"

#include "device/device.h"

namespace tt::umd {

device_collection device_collection::host_devices() {
    std::vector<std::shared_ptr<device>> devices;

    // TODO: Read cluster description to initialize ethernet devices
    auto chip_ids = kmd::scan();

    for (auto chip_id : chip_ids) {
        auto kmd = kmd::open(chip_id);
        if (kmd) {
            auto dev = pci_device::open(std::move(kmd), chip_id);

            if (dev) {
                devices.push_back(std::move(dev));
            }
        }
    }

    return {std::move(devices)};
}

std::shared_ptr<device> device_collection::get_device(uint32_t index) {
    if (index < devices.size()) {
        return devices[index];
    }

    return {};
}

void device_collection::setup_core_to_tlb_map(std::function<std::function<std::int32_t(xy_pair)>(architecture)> mapping_function) {
    for (const auto& device : devices) {
        if (device) {
            std::shared_ptr<pci_device> pci_device = std::dynamic_pointer_cast<tt::umd::pci_device>(device);

            if (pci_device) {
                pci_device->setup_core_to_tlb_map(mapping_function(pci_device->get_architecture()));
            }
        }
    }
}

}  // namespace tt::umd
