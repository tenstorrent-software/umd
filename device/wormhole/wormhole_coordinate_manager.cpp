/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "umd/device/wormhole_coordinate_manager.h"

using namespace tt::umd;

void WormholeCoordinateManager::fill_tensix_logical_to_translated() {
    size_t num_harvested_y = __builtin_popcount(CoordinateManager::tensix_harvesting_mask);

    for (size_t y = 0; y < CoordinateManager::tensix_grid_size.y - num_harvested_y; y++) {
        for (size_t x = 0; x < CoordinateManager::tensix_grid_size.x; x++) {
            const size_t translated_x = x + tensix_translated_coordinate_start_x;
            const size_t translated_y = y + tensix_translated_coordinate_start_y;
            CoordinateManager::tensix_logical_to_translated[{x, y}] =
                CoreCoord(translated_x, translated_y, CoreType::TENSIX, CoordSystem::TRANSLATED);
            CoordinateManager::tensix_translated_to_logical[tt_xy_pair(translated_x, translated_y)] =
                CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
        }
    }
}

void WormholeCoordinateManager::fill_eth_logical_to_translated() {
    for (size_t x = 0; x < CoordinateManager::eth_grid_size.x; x++) {
        for (size_t y = 0; y < CoordinateManager::eth_grid_size.y; y++) {
            const size_t translated_x = x + eth_translated_coordinate_start_x;
            const size_t translated_y = y + eth_translated_coordinate_start_y;
            CoordinateManager::eth_logical_to_translated[{x, y}] =
                CoreCoord(translated_x, translated_y, CoreType::ETH, CoordSystem::TRANSLATED);
            CoordinateManager::eth_translated_to_logical[{translated_x, translated_y}] =
                CoreCoord(x, y, CoreType::ETH, CoordSystem::LOGICAL);
        }
    }
}
