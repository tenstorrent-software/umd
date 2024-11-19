/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "device/coordinate_manager.h"

class GrayskullCoordinateManager : public CoordinateManager {

public:
    GrayskullCoordinateManager(const tt_xy_pair& worker_grid_size, const std::vector<tt_xy_pair>& workers, std::size_t harvesting_mask)
        : CoordinateManager(worker_grid_size, workers, harvesting_mask) {}

protected:
    CoreCoord translated_to_logical_tensix(const CoreCoord core_coord) override;
    CoreCoord logical_to_translated_tensix(const CoreCoord core_coord) override;
};
