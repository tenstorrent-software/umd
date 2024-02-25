// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "device/tt_silicon_driver_common.hpp"
#include "tt_xy_pair.h"
#include "tt_device.h"

std::optional<std::tuple<std::uint32_t, std::uint32_t>> tt_SiliconDevice::describe_tlb(tt_xy_pair coord) {
    auto tlb_index = map_core_to_tlb(tt_xy_pair(coord.x, coord.y));
    if (tlb_index < 0) {
        return std::nullopt;
    }

    return architecture_implementation->describe_tlb(tlb_index);
}

std::string TensixSoftResetOptionsToString(TensixSoftResetOptions value) {
    std::string output;

    if((value & TensixSoftResetOptions::BRISC) != TensixSoftResetOptions::NONE) {
        output += "BRISC | ";
    }
    if((value & TensixSoftResetOptions::TRISC0) != TensixSoftResetOptions::NONE) {
        output += "TRISC0 | ";
    }
    if((value & TensixSoftResetOptions::TRISC1) != TensixSoftResetOptions::NONE) {
        output += "TRISC1 | ";
    }
    if((value & TensixSoftResetOptions::TRISC2) != TensixSoftResetOptions::NONE) {
        output += "TRISC2 | ";
    }
    if((value & TensixSoftResetOptions::NCRISC) != TensixSoftResetOptions::NONE) {
        output += "NCRISC | ";
    }
    if((value & TensixSoftResetOptions::STAGGERED_START) != TensixSoftResetOptions::NONE) {
        output += "STAGGERED_START | ";
    }

  if(output.empty()) {
    output = "UNKNOWN";
  } else {
    output.erase(output.end() - 3, output.end());
  }

  return output;
}

