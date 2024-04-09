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

    // Orderings
    static constexpr uint64_t Relaxed = 0;
    static constexpr uint64_t Strict  = 1;
    static constexpr uint64_t Posted  = 2;

    bool check(const tlb_offsets & offset) const;
    std::optional<uint64_t> apply_offset(const tlb_offsets& offset) const;
};

constexpr inline bool operator==(const tlb_data& lhs, const tlb_data& rhs) {
    return lhs.local_offset == rhs.local_offset && lhs.x_end == rhs.x_end && lhs.y_end == rhs.y_end &&
           lhs.x_start == rhs.x_start && lhs.y_start == rhs.y_start && lhs.noc_sel == rhs.noc_sel &&
           lhs.mcast == rhs.mcast && lhs.ordering == rhs.ordering && lhs.linked == rhs.linked &&
           lhs.static_vc == rhs.static_vc;
}

constexpr inline bool operator!=(const tlb_data& lhs, const tlb_data& rhs) {
    return !(lhs == rhs);
}

struct tlb_configuration {
    uint32_t size;
    uint32_t base;
    uint32_t cfg_addr;
    uint32_t index_offset;
    tlb_offsets offset;
};

}  // namespace tt::umd
