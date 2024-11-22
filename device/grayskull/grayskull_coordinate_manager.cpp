/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "grayskull_coordinate_manager.h"

void GrayskullCoordinateManager::fill_tensix_logical_to_translated() {
    std::size_t num_harvested_y = __builtin_popcount(CoordinateManager::tensix_harvesting_mask);

    CoordinateManager::tensix_logical_to_translated.resize(CoordinateManager::tensix_grid_size.x);
    for (auto& vec : CoordinateManager::tensix_logical_to_translated) {
        vec.resize(CoordinateManager::tensix_grid_size.y - num_harvested_y);
    }

    for (std::size_t x = 0; x < CoordinateManager::tensix_grid_size.x; x++) {
        for (std::size_t y = 0; y < CoordinateManager::tensix_grid_size.y - num_harvested_y; y++) {
            const CoreCoord physical_coord = CoordinateManager::tensix_logical_to_physical[x][y];
            const std::size_t translated_x = physical_coord.x;
            const std::size_t translated_y = physical_coord.y;
            CoordinateManager::tensix_logical_to_translated[x][y] = CoreCoord(translated_x, translated_y, CoreType::TENSIX, CoordSystem::TRANSLATED);
            CoordinateManager::tensix_translated_to_logical[tt_xy_pair(translated_x, translated_y)] = CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
        }
    }
}