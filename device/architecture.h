/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <string>

namespace tt::umd {

/**
 * @brief architecture Enums
 */
enum class architecture {
    jawbridge = 0,
    grayskull = 1,
    wormhole = 2,
    wormhole_b0 = 3,
    blackhole = 4,
    invalid = 0xFF,
};

inline std::string to_string(architecture value) {
    switch(value) {
        case architecture::jawbridge:
            return "jawbridge";
        case architecture::grayskull:
            return "grayskull";
        case architecture::wormhole:
            return "wormhole";
        case architecture::wormhole_b0:
            return "wormhole_b0";
        case architecture::blackhole:
            return "blackhole";
        case architecture::invalid:
            return "invalid";
        default:
            return "<unknown>-" + std::to_string(static_cast<int>(value));
    }
}

}  // namespace tt::umd

namespace std {
    inline std::string to_string(tt::umd::architecture value) {
        return tt::umd::to_string(value);
    }
}

inline std::ostream& operator<<(std::ostream& os, tt::umd::architecture value) {
    return os << tt::umd::to_string(value);
}
