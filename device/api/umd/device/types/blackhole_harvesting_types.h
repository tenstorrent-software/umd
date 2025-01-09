/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>

namespace tt::umd {

struct telemetry_entry {
  uint16_t tag;
  uint16_t offset;
};

// struct telemetry_table {
//   uint32_t version;
//   uint32_t entry_count;
//   struct telemetry_entry tag_table[TELEM_ENUM_COUNT];
//   uint32_t telemetry[TELEM_ENUM_COUNT];
// };

}