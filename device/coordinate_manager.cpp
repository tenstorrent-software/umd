/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "device/coordinate_manager.h"
#include <memory>
#include <stdexcept>
#include "coordinate_manager.h"
#include "grayskull/grayskull_coordinate_manager.h"
#include "grayskull/grayskull_implementation.h"
#include "tt_core_coordinates.h"
#include "device/tt_xy_pair.h"

std::vector<std::vector<CoreCoord>>& CoordinateManager::get_logical_to_translated(CoreType core_type) {
    switch (core_type) {
        case CoreType::TENSIX:
            return tensix_logical_to_translated;
        case CoreType::DRAM:
            return dram_logical_to_translated;
        default:
            throw std::runtime_error("Core type is not supported for getting logical to translated mapping");
    }
}

std::vector<std::vector<CoreCoord>>& CoordinateManager::get_logical_to_virtual(CoreType core_type) {
    switch (core_type) {
        case CoreType::TENSIX:
            return tensix_logical_to_virtual;
        case CoreType::DRAM:
            return dram_logical_to_virtual;
        default:
            throw std::runtime_error("Core type is not supported for getting logical to virtual mapping");
    }
}

std::vector<std::vector<CoreCoord>>& CoordinateManager::get_logical_to_physical(CoreType core_type) {
    switch (core_type) {
        case CoreType::TENSIX:
            return tensix_logical_to_physical;
        case CoreType::DRAM:
            return dram_logical_to_physical;
        default:
            throw std::runtime_error("Core type is not supported for getting logical to physical mapping");
    }
}

std::map<tt_xy_pair, CoreCoord>& CoordinateManager::get_physical_to_logical(CoreType core_type) {
    switch (core_type) {
        case CoreType::TENSIX:
            return tensix_physical_to_logical;
        case CoreType::DRAM:
            return dram_physical_to_logical;
        default:
            throw std::runtime_error("Core type is not supported for getting physical to logical mapping");
    }
}

std::map<tt_xy_pair, CoreCoord>& CoordinateManager::get_virtual_to_logical(CoreType core_type) {
    switch (core_type) {
        case CoreType::TENSIX:
            return tensix_virtual_to_logical;
        case CoreType::DRAM:
            return dram_virtual_to_logical;
        default:
            throw std::runtime_error("Core type is not supported for getting virtual to logical mapping");
    }
}

std::map<tt_xy_pair, CoreCoord>& CoordinateManager::get_translated_to_logical(CoreType core_type) {
    switch (core_type) {
        case CoreType::TENSIX:
            return tensix_translated_to_logical;
        case CoreType::DRAM:
            return dram_translated_to_logical;
        default:
            throw std::runtime_error("Core type is not supported for getting translated to logical mapping");
    }
}

CoreCoord CoordinateManager::to_physical(const CoreCoord core_coord) {
    switch (core_coord.coord_system) {
        case CoordSystem::PHYSICAL:
            return core_coord;
        case CoordSystem::VIRTUAL:
        case CoordSystem::TRANSLATED:
            return to_physical(to_logical(core_coord));
    }
    
    // Coord system is surely logical.
    auto& logical_mapping = get_logical_to_physical(core_coord.core_type);
    return logical_mapping[core_coord.x][core_coord.y]; 
}

CoreCoord CoordinateManager::to_virtual(const CoreCoord core_coord) {
    switch (core_coord.coord_system) {
        case CoordSystem::TRANSLATED:
        case CoordSystem::PHYSICAL:
            return to_virtual(to_logical(core_coord));
        case CoordSystem::VIRTUAL:
            return core_coord;
    }

    // Coord system is surely logical.
    auto& logical_mapping = get_logical_to_virtual(core_coord.core_type);
    return logical_mapping[core_coord.x][core_coord.y];
}

CoreCoord CoordinateManager::to_logical(const CoreCoord core_coord) {
    switch (core_coord.coord_system) {
        case CoordSystem::LOGICAL:
            return core_coord;
        case CoordSystem::PHYSICAL: {
            auto& physical_mapping = get_physical_to_logical(core_coord.core_type);
            return physical_mapping[{core_coord.x, core_coord.y}];
        }
        case CoordSystem::VIRTUAL: {
            auto& virtual_mapping = get_virtual_to_logical(core_coord.core_type);
            return virtual_mapping[{core_coord.x, core_coord.y}];
        }
        case CoordSystem::TRANSLATED: {
            auto& translated_mapping = get_logical_to_translated(core_coord.core_type);
            return translated_mapping[core_coord.x][core_coord.y];
        }
    }
}

CoreCoord CoordinateManager::to_translated(const CoreCoord core_coord) {
    switch (core_coord.coord_system) {
        case CoordSystem::PHYSICAL:
        case CoordSystem::VIRTUAL:
            return to_translated(to_logical(core_coord));
        case CoordSystem::TRANSLATED:
            return core_coord;
    }

    // Coord system is surely logical.
    auto& logical_mapping = get_logical_to_translated(core_coord.core_type);
    return logical_mapping[core_coord.x][core_coord.y];
}

void CoordinateManager::clear_tensix_harvesting_structures() {
    tensix_logical_to_physical.clear();
    tensix_logical_to_virtual.clear();
    tensix_physical_to_logical.clear();
    tensix_virtual_to_logical.clear();
    tensix_logical_to_translated.clear();
    tensix_translated_to_logical.clear();
}

void CoordinateManager::clear_dram_harvesting_structures() {
    dram_logical_to_virtual.clear();
    dram_logical_to_physical.clear();
    dram_virtual_to_logical.clear();
    dram_physical_to_logical.clear();
    dram_logical_to_translated.clear();
    dram_translated_to_logical.clear();
}
#include <iostream>
void CoordinateManager::tensix_harvesting(const std::size_t harvesting_mask) {
    // clear_tensix_harvesting_structures();

    // std::size_t num_harvested_y = __builtin_popcount(harvesting_mask);
    // std::size_t grid_size_x = tensix_grid_size.x;
    // std::size_t grid_size_y = tensix_grid_size.y;
    
    // tensix_logical_to_physical.resize(grid_size_x);
    // for (auto& vec : tensix_logical_to_physical) {
    //     vec.resize(grid_size_y - num_harvested_y);
    // }

    // std::cout << "here" << std::endl;

    // std::size_t logical_x = 0;
    // for (std::size_t x = 0; x < grid_size_x; x++) {
    //     if (!(harvesting_mask & (1 << x))) {
    //         for (std::size_t y = 0; y < grid_size_y; y++) {
    //             const tt_xy_pair& tensix_core = tensix_cores[x * grid_size_y + y];
    //             tensix_logical_to_physical[logical_x][y] = CoreCoord(tensix_core.x, tensix_core.y, CoreType::TENSIX, CoordSystem::PHYSICAL);
    //             tensix_physical_to_logical[tensix_core] = CoreCoord(logical_x, y, CoreType::TENSIX, CoordSystem::PHYSICAL);
    //         }
    //         logical_x++;
    //     }
    // }

    // std::cout << "here 2" << std::endl;

    // tensix_logical_to_virtual.resize(grid_size_x);
    // for (auto& vec : tensix_logical_to_virtual) {
    //     vec.resize(grid_size_y - num_harvested_y);
    // }

    // for (std::size_t x = 0; x < grid_size_x; x++) {
    //     for (std::size_t y = 0; y < grid_size_y - num_harvested_y; y++) {
    //         const tt_xy_pair& tensix_core = tensix_cores[x * grid_size_y + y];
    //         tensix_logical_to_virtual[x][y] = CoreCoord(tensix_core.x, tensix_core.y, CoreType::TENSIX, CoordSystem::VIRTUAL);
    //         tensix_virtual_to_logical[tensix_core] = CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
    //     }
    // }

    // std::cout << "here 3" << std::endl;

    // fill_tensix_logical_to_translated();

    // std::cout << "here 4" << std::endl;
}

void CoordinateManager::fill_tensix_logical_to_translated() {}

void CoordinateManager::dram_harvesting(const std::size_t dram_harvesting_mask) {

    // std::cout << "harvesting dram" << std::endl;

    // dram_logical_to_physical.resize(dram_grid_size.x);
    // for (auto& vec : dram_logical_to_physical) {
    //     vec.resize(dram_grid_size.y);
    // }

    // dram_logical_to_virtual.resize(dram_grid_size.x);
    // for (auto& vec : dram_logical_to_virtual) {
    //     vec.resize(dram_grid_size.y);
    // }

    // for (std::size_t x = 0; x < dram_grid_size.x; x++) {
    //     for (std::size_t y = 0; y < dram_grid_size.y; y++) {
    //         const tt_xy_pair& dram_core = dram_cores[x * dram_grid_size.y + y];
    //         dram_logical_to_virtual[x][y] = CoreCoord(dram_core.x, dram_core.y, CoreType::DRAM, CoordSystem::VIRTUAL);
    //         dram_virtual_to_logical[dram_core] = CoreCoord(x, y, CoreType::DRAM, CoordSystem::LOGICAL);

    //         dram_logical_to_physical[x][y] = CoreCoord(dram_core.x, dram_core.y, CoreType::DRAM, CoordSystem::PHYSICAL);
    //         dram_physical_to_logical[dram_core] = CoreCoord(x, y, CoreType::DRAM, CoordSystem::LOGICAL);
    //     }
    // }

    // std::cout << "finished harvesting dram" << std::endl;
}

#include "device/grayskull/grayskull_coordinate_manager.h"
#include "device/wormhole/wormhole_coordinate_manager.h"
#include "device/blackhole/blackhole_coordinate_manager.h"

std::unique_ptr<CoordinateManager> CoordinateManager::get_coordinate_manager(
    tt::ARCH arch,
    const tt_xy_pair& worker_grid_size,
    const std::vector<tt_xy_pair>& workers,
    const std::size_t tensix_harvesting_mask,
    const std::size_t dram_harvesting_mask) {

    switch (arch) {
        case tt::ARCH::GRAYSKULL:
            return std::make_unique<GrayskullCoordinateManager>(worker_grid_size, workers, tensix_harvesting_mask, dram_harvesting_mask);
        case tt::ARCH::WORMHOLE_B0:
            return std::make_unique<WormholeCoordinateManager>(worker_grid_size, workers, tensix_harvesting_mask, dram_harvesting_mask);
        case tt::ARCH::BLACKHOLE:
            return std::make_unique<BlackholeCoordinateManager>(worker_grid_size, workers, tensix_harvesting_mask, dram_harvesting_mask);
        case tt::ARCH::Invalid:
            throw std::runtime_error("Invalid architecture for creating coordinate manager");
    }

    throw std::runtime_error("Invalid architecture for creating coordinate manager");
}
