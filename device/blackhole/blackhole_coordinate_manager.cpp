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
