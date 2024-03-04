/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>
#include <vector>

#include "device/architecture.h"
#include "device/device.h"
#include "device/kmd.h"

namespace tt::umd {

class device_collection {
   protected:
    device_collection(std::vector<std::shared_ptr<device>> devices) : devices(std::move(devices)) {}

   public:
    virtual ~device_collection() = default;

    static device_collection host_devices();

    // Returns device from collection if it is initialized; nullptr otherwise.
    std::shared_ptr<device> get_device(chip_id_t logical_device_id);

    // Returns devices inside this device collection.
    // Index represents logical device id.
    // Note that all logical device ids will contain loaded device. It means that some elements can be nullptr.
    const std::vector<std::shared_ptr<device>>& get_devices() const { return devices; }

    // Setup core to tlb mapping function for all pci devices in collection.
    void setup_core_to_tlb_map(std::function<std::function<std::int32_t(xy_pair)>(architecture)> mapping_function);

   private:
    std::vector<std::shared_ptr<device>> devices;
};

}  // namespace tt::umd
