/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <vector>

#include "device/device.h"

namespace tt::umd {

class device_collection {
   protected:
    device_collection(std::vector<std::shared_ptr<device>> devices) : devices(std::move(devices)) {}

   public:
    virtual ~device_collection() = default;

    static device_collection host_devices();

    std::shared_ptr<device> get_device(uint32_t index);
    const std::vector<std::shared_ptr<device>>& get_devices() const { return devices; }

   protected:
    std::vector<std::shared_ptr<device>> devices;
};

}  // namespace tt::umd
