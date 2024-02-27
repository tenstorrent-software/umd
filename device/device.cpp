// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "device/device.h"

namespace tt::umd {

device::device() {
    // TODO: Implement
}

device::~device() {
    // TODO: Implement
}

std::unique_ptr<device> device::open(std::unique_ptr<tt::umd::kmd> kmd) {
    // TODO: Implement
    return {};
}

}  // namespace tt::umd
