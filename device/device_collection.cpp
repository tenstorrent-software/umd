// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "device/device_collection.h"

namespace tt::umd {

device_collection device_collection::host_devices() {
    std::vector<std::unique_ptr<device>> devices;

    // TODO: Read cluster description to initialize ethernet devices
    auto chip_ids = kmd::scan();

    for (auto chip_id : chip_ids) {
        auto kmd = kmd::open(chip_id);
        if (kmd) {
            auto dev = device::open(std::move(kmd));

            if (dev) {
                devices.push_back(std::move(dev));
            }
        }
    }

    return {std::move(devices)};
}

}  // namespace tt::umd
