/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <memory>

#include "device/architecture_implementation.h"
#include "device/kmd.h"

namespace tt::umd {

class device {
   private:
    device(const device&) = delete;
    device(device&&) = delete;
    device& operator=(const device&) = delete;
    device& operator=(device&&) = delete;

   public:
    device();
    virtual ~device();

    static std::unique_ptr<device> open(std::unique_ptr<tt::umd::kmd> kmd);

    const architecture_implementation* get_architecture_implementation() const {
        return arch_implementation.get();
    }

   protected:
    std::unique_ptr<architecture_implementation> arch_implementation;

   private:
    std::unique_ptr<tt::umd::kmd> kmd;
};

}  // namespace tt::umd
