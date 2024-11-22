/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <map>
#include <vector>
#include <set>

#include "device/tt_xy_pair.h"
#include "device/tt_arch_types.h"
#include "tt_core_coordinates.h"

class CoordinateManager {

public:
    CoordinateManager(
        const tt_xy_pair& worker_grid_size, const std::vector<tt_xy_pair>& workers, const std::size_t tensix_harvesting_mask,
        const tt_xy_pair& dram_grid_size, const std::vector<tt_xy_pair>& dram_cores, const std::size_t dram_harvesting_mask)
        : tensix_grid_size(worker_grid_size), tensix_cores(workers), tensix_harvesting_mask(tensix_harvesting_mask),
          dram_grid_size(dram_grid_size), dram_cores(dram_cores), dram_harvesting_mask(dram_harvesting_mask)
        {}

    virtual void tensix_harvesting(const std::size_t harvesting_mask);

    virtual void dram_harvesting(const std::size_t dram_harvesting_mask);

    static std::unique_ptr<CoordinateManager> get_coordinate_manager(
        tt::ARCH arch,
        const tt_xy_pair& worker_grid_size,
        const std::vector<tt_xy_pair>& workers,
        const std::size_t tensix_harvesting_mask,
        const std::size_t dram_harvesting_mask);

    CoordinateManager(CoordinateManager& other) = default;

    CoreCoord to_physical(const CoreCoord core_coord);
    CoreCoord to_logical(const CoreCoord core_coord);
    CoreCoord to_virtual(const CoreCoord core_coord);
    CoreCoord to_translated(const CoreCoord core_coord);

    virtual ~CoordinateManager() = default;

protected:
    void clear_tensix_harvesting_structures();
    void clear_dram_harvesting_structures();

    // TODO(pjanevski): this should be abstract function.
    virtual void fill_tensix_logical_to_translated();

    std::vector<std::vector<CoreCoord>> tensix_logical_to_translated;
    std::vector<std::vector<CoreCoord>> tensix_logical_to_virtual;
    std::vector<std::vector<CoreCoord>> tensix_logical_to_physical;
    std::map<tt_xy_pair, CoreCoord> tensix_physical_to_logical;
    std::map<tt_xy_pair, CoreCoord> tensix_virtual_to_logical;
    std::map<tt_xy_pair, CoreCoord> tensix_translated_to_logical;

    std::vector<std::vector<CoreCoord>> dram_logical_to_translated;
    std::vector<std::vector<CoreCoord>> dram_logical_to_virtual;
    std::vector<std::vector<CoreCoord>> dram_logical_to_physical;
    std::map<tt_xy_pair, CoreCoord> dram_physical_to_logical;
    std::map<tt_xy_pair, CoreCoord> dram_virtual_to_logical;
    std::map<tt_xy_pair, CoreCoord> dram_translated_to_logical;

    std::vector<std::vector<CoreCoord>>& get_logical_to_translated(CoreType core_type);
    std::vector<std::vector<CoreCoord>>& get_logical_to_virtual(CoreType core_type);
    std::vector<std::vector<CoreCoord>>& get_logical_to_physical(CoreType core_type);

    std::map<tt_xy_pair, CoreCoord>& get_physical_to_logical(CoreType core_type);
    std::map<tt_xy_pair, CoreCoord>& get_virtual_to_logical(CoreType core_type);
    std::map<tt_xy_pair, CoreCoord>& get_translated_to_logical(CoreType core_type);

    const tt_xy_pair tensix_grid_size;
    const std::vector<tt_xy_pair>& tensix_cores;
    const std::size_t tensix_harvesting_mask;

    // TODO(pjanevski): put const for this attributes
    const tt_xy_pair dram_grid_size;
    const std::vector<tt_xy_pair> dram_cores;
    const std::size_t dram_harvesting_mask;
};
