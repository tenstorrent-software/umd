/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "device/coordinate_manager.h"
#include "device/grayskull/grayskull_implementation.h"
#include "grayskull_implementation.h"

class GrayskullCoordinateManager : public CoordinateManager {

public:
    GrayskullCoordinateManager(const tt_xy_pair& worker_grid_size, const std::vector<tt_xy_pair>& workers, const std::size_t tensix_harvesting_mask, const std::size_t dram_harvesting_mask)
        : CoordinateManager(worker_grid_size, workers, tensix_harvesting_mask,
                            {tt::umd::grayskull::NUM_DRAM_BANKS, tt::umd::grayskull::NUM_NOC_PORTS_PER_DRAM_BANK}, tt::umd::grayskull::DRAM_CORES, dram_harvesting_mask) {}

protected:
    CoreCoord translated_to_logical_tensix(const CoreCoord core_coord) override;
    CoreCoord logical_to_translated_tensix(const CoreCoord core_coord) override;
};
