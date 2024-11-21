/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "blackhole_coordinate_manager.h"

std::set<std::size_t> BlackholeCoordinateManager::get_x_coordinates_to_harvest(std::size_t harvesting_mask) {
    std::set<std::size_t> x_to_harvest;
    std::size_t logical_x = 0;
    while (harvesting_mask > 0) {
        if (harvesting_mask & 1) {
            x_to_harvest.insert(logical_x);
        }
        logical_x++;
        harvesting_mask >>= 1;
    }
    return x_to_harvest;
}

CoreCoord BlackholeCoordinateManager::translated_to_logical_tensix(const CoreCoord core_coord) {
    const CoreCoord virtual_coord = CoreCoord(core_coord.x, core_coord.y, CoreType::TENSIX, CoordSystem::VIRTUAL);
    return CoordinateManager::to_logical(virtual_coord);
}

CoreCoord BlackholeCoordinateManager::logical_to_translated_tensix(const CoreCoord core_coord) {
    const CoreCoord virtual_coord = CoordinateManager::to_virtual(core_coord);
    return CoreCoord(virtual_coord.x, virtual_coord.y, CoreType::TENSIX, CoordSystem::TRANSLATED);
}

void BlackholeCoordinateManager::dram_harvesting(const std::size_t dram_harvesting_mask) {
    
    std::size_t get_num_harvested_x = __builtin_popcount(dram_harvesting_mask);

    for (std::size_t x = 0; x < dram_grid_size.x - get_num_harvested_x; x++) {
        for (std::size_t y = 0; y < dram_grid_size.y; y++) {
            CoordinateManager::dram_logical_to_virtual[{x, y}] = CoordinateManager::dram_cores[x * dram_grid_size.y + y];
            CoordinateManager::dram_virtual_to_logical[dram_cores[x * dram_grid_size.y + y]] = {x, y};
        }
    }
    
    std::size_t logical_x = 0;
    for (std::size_t x = 0; x < dram_grid_size.x; x++) {
        if (!(dram_harvesting_mask & (1 << x))) {
            for (std::size_t y = 0; y < dram_grid_size.y; y++) {
                CoordinateManager::dram_logical_to_physical[{logical_x , y}] = CoordinateManager::dram_cores[x * dram_grid_size.y + y];
                CoordinateManager::dram_physical_to_logical[dram_cores[x * dram_grid_size.y + y]] = {logical_x, y};
            }
            logical_x++;
        }
    }
}