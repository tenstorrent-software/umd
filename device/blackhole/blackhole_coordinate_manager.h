/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "device/coordinate_manager.h"
#include "device/blackhole/blackhole_implementation.h"

class BlackholeCoordinateManager : public CoordinateManager {

public:
    BlackholeCoordinateManager(const tt_xy_pair& worker_grid_size, const std::vector<tt_xy_pair>& workers, const std::size_t tensix_harvesting_mask, const std::size_t dram_harvesting_mask)
        : CoordinateManager(worker_grid_size, workers, tensix_harvesting_mask,
                            {tt::umd::blackhole::NUM_DRAM_BANKS, tt::umd::blackhole::NUM_NOC_PORTS_PER_DRAM_BANK}, tt::umd::blackhole::DRAM_CORES, dram_harvesting_mask) {}

    void dram_harvesting(const std::size_t dram_harvesting_mask) override;

protected:
    CoreCoord translated_to_logical_tensix(const CoreCoord core_coord) override;
    CoreCoord logical_to_translated_tensix(const CoreCoord core_coord) override;

    std::set<std::size_t> get_x_coordinates_to_harvest(std::size_t harvesting_mask) override;
};
