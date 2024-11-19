/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "grayskull_coordinate_manager.h"

CoreCoord GrayskullCoordinateManager::translated_to_logical_tensix(const CoreCoord core_coord) {
    const CoreCoord physical_coord = CoreCoord(core_coord.x, core_coord.y, CoreType::TENSIX, CoordSystem::PHYSICAL);
    return CoordinateManager::to_logical(physical_coord);
}

CoreCoord GrayskullCoordinateManager::logical_to_translated_tensix(const CoreCoord core_coord) {
    const CoreCoord physical_coord = CoordinateManager::to_physical(core_coord);
    return CoreCoord(physical_coord.x, physical_coord.y, CoreType::TENSIX, CoordSystem::TRANSLATED);
}