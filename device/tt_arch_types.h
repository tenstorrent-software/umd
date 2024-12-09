/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

namespace tt {

/**
 * @brief ARCH Enums
 */
enum class ARCH {
    GRAYSKULL = 1,
    WORMHOLE_B0 = 2,
    BLACKHOLE = 3,
    QUASAR = 4,
    Invalid = 0xFF,
};
}

