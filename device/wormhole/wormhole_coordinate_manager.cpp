/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "wormhole_coordinate_manager.h"

std::set<std::size_t> WormholeCoordinateManager::get_y_coordinates_to_harvest(std::size_t harvesting_mask) {
    std::set<std::size_t> y_to_harvest;
    std::size_t logical_y = 0;
    while (harvesting_mask > 0) {
        if (harvesting_mask & 1) {
            y_to_harvest.insert(logical_y);
        }
        logical_y++;
        harvesting_mask >>= 1;
    }
    return y_to_harvest;
}

CoreCoord WormholeCoordinateManager::translated_to_logical_tensix(const CoreCoord core_coord) {
    return CoreCoord{core_coord.x - translated_coordinate_start_x, core_coord.y - translated_coordinate_start_y, CoreType::TENSIX, CoordSystem::LOGICAL};
}

CoreCoord WormholeCoordinateManager::logical_to_translated_tensix(const CoreCoord core_coord) {
    return CoreCoord{core_coord.x + translated_coordinate_start_x, core_coord.y + translated_coordinate_start_y, CoreType::TENSIX, CoordSystem::TRANSLATED};
}