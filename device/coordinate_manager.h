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
    CoordinateManager(const tt_xy_pair& worker_grid_size, const std::vector<tt_xy_pair>& workers, std::size_t harvesting_mask)
        : worker_grid_size(worker_grid_size), workers(workers), harvesting_mask(harvesting_mask) {}

    virtual void perform_harvesting(std::size_t harvesting_mask);

    static std::unique_ptr<CoordinateManager> get_coordinate_manager(
        tt::ARCH arch,
        const tt_xy_pair& worker_grid_size,
        const std::vector<tt_xy_pair>& workers,
        std::size_t harvesting_mask);

    CoordinateManager(CoordinateManager& other) = default;

    CoreCoord to_physical(const CoreCoord core_coord);
    CoreCoord to_logical(const CoreCoord core_coord);
    CoreCoord to_virtual(const CoreCoord core_coord);
    CoreCoord to_translated(const CoreCoord core_coord);

    virtual ~CoordinateManager() = default;

protected:
    virtual void clear_harvesting_structures();
    
    virtual std::set<std::size_t> get_x_coordinates_to_harvest(std::size_t harvesting_mask);
    virtual std::set<std::size_t> get_y_coordinates_to_harvest(std::size_t harvesting_mask);

    virtual void fill_logical_to_physical_mapping(
        const std::set<size_t>& x_to_harvest, const std::set<size_t>& y_to_harvest,
        const std::set<size_t>& physical_x_unharvested, const std::set<size_t>& physical_y_unharvested);
    virtual void fill_logical_to_virtual_mapping(const std::set<size_t>& physical_x_unharvested, const std::set<size_t>& physical_y_unharvested);
    
    CoreCoord to_tensix_physical(const CoreCoord core_coord);
    CoreCoord to_tensix_logical(const CoreCoord core_coord);
    CoreCoord to_tensix_virtual(const CoreCoord core_coord);
    CoreCoord to_tensix_translated(const CoreCoord core_coord);

    // TODO(pjanevski): this should be abstract functions
    // Making deep copy of SocDescriptor is harded if these are abstract
    virtual CoreCoord translated_to_logical_tensix(const CoreCoord core_coord);
    virtual CoreCoord logical_to_translated_tensix(const CoreCoord core_coord);

    std::map<std::size_t, std::size_t> physical_y_to_logical_y;
    std::map<std::size_t, std::size_t> physical_x_to_logical_x;

    std::vector<std::size_t> logical_y_to_physical_y;
    std::vector<std::size_t> logical_x_to_physical_x;

    std::vector<std::size_t> logical_y_to_virtual_y;
    std::vector<std::size_t> logical_x_to_virtual_x;

    std::map<std::size_t, std::size_t> virtual_y_to_logical_y;
    std::map<std::size_t, std::size_t> virtual_x_to_logical_x;

    const tt_xy_pair worker_grid_size;
    const std::vector<tt_xy_pair>& workers;
    const std::size_t harvesting_mask;
};
