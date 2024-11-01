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

tt_translated_coords WormholeCoordinateManager::to_translated_coords(tt_logical_coords logical_coords) {
    return tt_translated_coords(logical_coords.x + translated_coordinate_start_x, logical_coords.y + translated_coordinate_start_y);
}

tt_logical_coords WormholeCoordinateManager::to_logical_coords(tt_translated_coords translated_coords) {
    return tt_logical_coords(translated_coords.x - translated_coordinate_start_x, translated_coords.y - translated_coordinate_start_y);
}

// General ways of how we translate coordinates is not a key part of this PR.
// Main things to look at here is the way we are using the functions to translate coordinates
// To understand how the translations works for V1 and V2, based on struct types.

// v1 functions
CoreCoord_V1 WormholeCoordinateManager::to_tensix_physical(CoreCoord_V1 core_coords) {
    CoreCoord_V1 physical_tensix_coords;
    physical_tensix_coords.coord_system = CoordSystem::PHYSICAL;
    physical_tensix_coords.core_type = CoreType::TENSIX;
    switch (core_coords.coord_system) {
        case CoordSystem::LOGICAL:
            physical_tensix_coords.x = logical_x_to_physical_x[core_coords.x];
            physical_tensix_coords.y = logical_y_to_physical_y[core_coords.y];
        case CoordSystem::PHYSICAL:
            return core_coords;
        case CoordSystem::VIRTUAL:
            physical_tensix_coords.x = logical_x_to_physical_x[virtual_x_to_logical_x[core_coords.x]];
            physical_tensix_coords.y = logical_y_to_physical_y[virtual_y_to_logical_y[core_coords.y]];
        case CoordSystem::TRANSLATED:
            physical_tensix_coords.x = logical_x_to_physical_x[core_coords.x - translated_coordinate_start_x];
            physical_tensix_coords.y = logical_y_to_physical_y[core_coords.y - translated_coordinate_start_y];
    }

    return physical_tensix_coords;
}

CoreCoord_V1 WormholeCoordinateManager::to_dram_physical(CoreCoord_V1 core_coords) {
    CoreCoord_V1 physical_dram_coords;
    physical_dram_coords.coord_system = CoordSystem::PHYSICAL;
    physical_dram_coords.core_type = CoreType::DRAM;
    switch(core_coords.coord_system) {
        case CoordSystem::LOGICAL:
            
        case CoordSystem::PHYSICAL:
            return core_coords;
        case CoordSystem::VIRTUAL:
            physical_dram_coords.x = core_coords.x;
            physical_dram_coords.y = core_coords.y;
        case CoordSystem::TRANSLATED:
            physical_dram_coords.x = core_coords.x;
            physical_dram_coords.y = core_coords.y;
    }
    
    return physical_dram_coords;
}

CoreCoord_V1 WormholeCoordinateManager::to_physical(CoreCoord_V1 core_coords) {
    switch (core_coords.core_type) {
        case CoreType::TENSIX:
            return to_tensix_physical(core_coords);
        case CoreType::DRAM:
            return to_dram_physical(core_coords);
    }

    throw std::runtime_error("Invalid core type");
}

// v2 functions
TensixCoreCoord_V2 WormholeCoordinateManager::to_physical(TensixCoreCoord_V2 tensix_coords) {
    TensixCoreCoord_V2 physical_tensix_coords;
    physical_tensix_coords.coord_system = CoordSystem::PHYSICAL;
    switch (tensix_coords.coord_system) {
        case CoordSystem::LOGICAL:
            physical_tensix_coords.x = logical_x_to_physical_x[tensix_coords.x];
            physical_tensix_coords.y = logical_y_to_physical_y[tensix_coords.y];
        case CoordSystem::PHYSICAL:
            return tensix_coords;
        case CoordSystem::VIRTUAL:
            physical_tensix_coords.x = logical_x_to_physical_x[virtual_x_to_logical_x[tensix_coords.x]];
            physical_tensix_coords.y = logical_y_to_physical_y[virtual_y_to_logical_y[tensix_coords.y]];
        case CoordSystem::TRANSLATED:
            physical_tensix_coords.x = logical_x_to_physical_x[tensix_coords.x - translated_coordinate_start_x];
            physical_tensix_coords.y = logical_y_to_physical_y[tensix_coords.y - translated_coordinate_start_y];
    }

    return physical_tensix_coords;
}

DramCoreCoord_V2 WormholeCoordinateManager::to_physical(DramCoreCoord_V2 dram_coords) {
    DramCoreCoord_V2 physical_dram_coords;
    physical_dram_coords.coord_system = CoordSystem::PHYSICAL;
    switch(dram_coords.coord_system) {
        case CoordSystem::LOGICAL:
            // TODO implement logical to physical translation for dram coordinates
            // but that for review it is not important how that logic is going to look like
            return dram_coords;
        case CoordSystem::PHYSICAL:
            return dram_coords;
        case CoordSystem::VIRTUAL:
            // virtual coords for DRAM same as physical for Wormhole.
            physical_dram_coords.x = dram_coords.x;
            physical_dram_coords.y = dram_coords.y;
        case CoordSystem::TRANSLATED:
            // translated coords for DRAM same as physical for Wormhole.
            physical_dram_coords.x = dram_coords.x;
            physical_dram_coords.y = dram_coords.y;
    }
    
    return physical_dram_coords;
}
