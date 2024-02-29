// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "device/device_collection.h"

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

}  // namespace tt::umd
