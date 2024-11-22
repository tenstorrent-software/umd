/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "blackhole_coordinate_manager.h"

void BlackholeCoordinateManager::tensix_harvesting(const std::size_t tensix_harvesting_mask) {
    CoordinateManager::clear_tensix_harvesting_structures();

    std::size_t num_harvested_x = __builtin_popcount(tensix_harvesting_mask);
    std::size_t grid_size_x = CoordinateManager::tensix_grid_size.x;
    std::size_t grid_size_y = CoordinateManager::tensix_grid_size.y;
    
    tensix_logical_to_physical.resize(grid_size_x - num_harvested_x);
    for (auto& vec : tensix_logical_to_physical) {
        vec.resize(grid_size_y);
    }

    std::size_t logical_y = 0;
    for (std::size_t y = 0; y < grid_size_y; y++) {
        if (!(tensix_harvesting_mask & (1 << y))) {
            for (std::size_t x = 0; x < grid_size_x; x++) {
                const tt_xy_pair& tensix_core = CoordinateManager::tensix_cores[x * grid_size_y + y];
                tensix_logical_to_physical[x][logical_y] = CoreCoord(tensix_core.x, tensix_core.y, CoreType::TENSIX, CoordSystem::PHYSICAL);
                tensix_physical_to_logical[tensix_core] = CoreCoord(x, logical_y, CoreType::TENSIX, CoordSystem::PHYSICAL);
            }
            logical_y++;
        }
    }

    tensix_logical_to_virtual.resize(grid_size_x - num_harvested_x);
    for (auto& vec : tensix_logical_to_virtual) {
        vec.resize(grid_size_y);
    }

    for (std::size_t x = 0; x < grid_size_x - num_harvested_x; x++) {
        for (std::size_t y = 0; y < grid_size_y; y++) {
            const tt_xy_pair& tensix_core = CoordinateManager::tensix_cores[x * grid_size_y + y];
            tensix_logical_to_virtual[x][y] = CoreCoord(tensix_core.x, tensix_core.y, CoreType::TENSIX, CoordSystem::VIRTUAL);
            tensix_virtual_to_logical[tensix_core] = CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
        }
    }

    BlackholeCoordinateManager::fill_tensix_logical_to_translated();
}

void BlackholeCoordinateManager::fill_tensix_logical_to_translated() {
    const std::size_t num_harvested_x = __builtin_popcount(CoordinateManager::tensix_harvesting_mask);
    const std::size_t grid_size_x = CoordinateManager::tensix_grid_size.x;
    const std::size_t grid_size_y = CoordinateManager::tensix_grid_size.y;
    
    CoordinateManager::tensix_logical_to_translated.resize(grid_size_x - num_harvested_x);
    for (auto& vec : CoordinateManager::tensix_logical_to_translated) {
        vec.resize(grid_size_y);
    }

    for (std::size_t x = 0; x < grid_size_x - num_harvested_x; x++) {
        for (std::size_t y = 0; y < grid_size_y; y++) {
            const CoreCoord virtual_coord = CoordinateManager::tensix_logical_to_virtual[x][y];
            const std::size_t translated_x = virtual_coord.x;
            const std::size_t translated_y = virtual_coord.y;
            CoordinateManager::tensix_logical_to_translated[x][y] = CoreCoord(translated_x, translated_y, CoreType::TENSIX, CoordSystem::TRANSLATED);
            CoordinateManager::tensix_translated_to_logical[tt_xy_pair(translated_x, translated_y)] = CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
        }
    }
}

void BlackholeCoordinateManager::dram_harvesting(const std::size_t dram_harvesting_mask) {
    std::size_t num_harvested_banks = __builtin_popcount(dram_harvesting_mask);

    CoordinateManager::dram_logical_to_physical.resize(dram_grid_size.x - num_harvested_banks);
    for (auto& vec : CoordinateManager::dram_logical_to_physical) {
        vec.resize(dram_grid_size.y);
    }

    CoordinateManager::dram_logical_to_virtual.resize(dram_grid_size.x - num_harvested_banks);
    for (auto& vec : CoordinateManager::dram_logical_to_virtual) {
        vec.resize(dram_grid_size.y);
    }

    for (std::size_t x = 0; x < dram_grid_size.x - num_harvested_banks; x++) {
        for (std::size_t y = 0; y < dram_grid_size.y; y++) {
            const tt_xy_pair& dram_core = CoordinateManager::dram_cores[x * dram_grid_size.y + y];
            CoordinateManager::dram_logical_to_virtual[x][y] = CoreCoord(dram_core.x, dram_core.y, CoreType::DRAM, CoordSystem::VIRTUAL);
            CoordinateManager::dram_virtual_to_logical[dram_core] = CoreCoord(x, y, CoreType::DRAM, CoordSystem::LOGICAL);
        }
    }
    
    std::size_t logical_x = 0;
    for (std::size_t x = 0; x < dram_grid_size.x; x++) {
        if (!(dram_harvesting_mask & (1 << x))) {
            for (std::size_t y = 0; y < dram_grid_size.y; y++) {
                const tt_xy_pair& dram_core = CoordinateManager::dram_cores[x * dram_grid_size.y + y];
                CoordinateManager::dram_logical_to_physical[logical_x][y] = CoreCoord(dram_core.x, dram_core.y, CoreType::DRAM, CoordSystem::PHYSICAL);
                CoordinateManager::dram_physical_to_logical[dram_core] = CoreCoord(logical_x, y, CoreType::DRAM, CoordSystem::LOGICAL);
            }
            logical_x++;
        }
    }
}