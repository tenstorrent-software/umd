/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstdint>
#include <optional>

namespace tt::umd {

struct tlb_offsets {
    uint32_t local_offset;
    uint32_t x_end;
    uint32_t y_end;
    uint32_t x_start;
    uint32_t y_start;
    uint32_t noc_sel;
    uint32_t mcast;
    uint32_t ordering;
    uint32_t linked;
    uint32_t static_vc;
    uint32_t static_vc_end;
};

enum class tlb_data_ordering : uint8_t {
    relaxed = 0,
    strict = 1,
    posted = 2,
};

struct tlb_data {
    uint64_t local_offset = 0;
    uint64_t x_end = 0;
    uint64_t y_end = 0;
    uint64_t x_start = 0;
    uint64_t y_start = 0;
    uint64_t noc_sel = 0;
    uint64_t mcast = 0;
    uint64_t ordering = 0;
    uint64_t linked = 0;
    uint64_t static_vc = 0;

    // TODO: Backward compability, remove in favor of tlb_data_ordering
    static constexpr uint64_t Relaxed = static_cast<uint64_t>(tlb_data_ordering::relaxed);
    static constexpr uint64_t Strict  = static_cast<uint64_t>(tlb_data_ordering::strict);
    static constexpr uint64_t Posted  = static_cast<uint64_t>(tlb_data_ordering::posted);

    bool check(const tlb_offsets & offset) const;
    std::optional<uint64_t> apply_offset(const tlb_offsets& offset) const;
};

struct tlb_configuration {
    uint32_t size;
    uint32_t base;
    uint32_t cfg_addr;
    uint32_t index_offset;
    tlb_offsets offset;
};

struct dynamic_tlb {
    uint32_t bar_offset;      // Offset that address is mapped to, within the PCI BAR.
    uint32_t remaining_size;  // Bytes remaining between bar_offset and end of the TLB.
};

}  // namespace tt::umd
