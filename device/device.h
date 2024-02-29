/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>

#include "device/architecture.h"
#include "device/architecture_implementation.h"
#include "device/kmd.h"
#include "device/xy_pair.h"

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
    chip_id_t get_logical_device_id() const { return logical_device_id; }

    virtual void read_from_device(void* memory, size_t size, cxy_pair core, uint64_t address) = 0;

   protected:
    device(architecture arch, chip_id_t logical_device_id);

    chip_id_t logical_device_id;
    architecture arch;
    std::unique_ptr<architecture_implementation> arch_impl;
};

class pci_device : public device {
   public:
    static std::shared_ptr<device> open(std::unique_ptr<tt::umd::kmd> kmd, chip_id_t logical_device_id);

    void read_from_device(void* memory, size_t size, cxy_pair core, uint64_t address) override;

   private:
    pci_device(std::unique_ptr<tt::umd::kmd> kmd, chip_id_t logical_device_id);

    void read_device_memory(void* memory, size_t size, cxy_pair core, uint64_t address);
    void read_device_registers(void* memory, size_t size, cxy_pair core, uint64_t address);

    std::unique_ptr<tt::umd::kmd> kmd;
};

// class network_device : public device {
//     // TODO: Implement
//     void read_from_device(void* memory, size_t size, cxy_pair core, uint64_t address) override;
// };

}  // namespace tt::umd
