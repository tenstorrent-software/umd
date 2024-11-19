/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

 #include "device/tt_xy_pair.h"


/*
 * CoordSystem is an enum class that represents all types of coordinate
 * systems that can be used to represent a core's location.
 */
enum class CoordSystem {
    LOGICAL,
    PHYSICAL,
    VIRTUAL,
    TRANSLATED,
};

/*
 * CoreType is an enum class that represents all types of cores
 * present on the Tenstorrent chip.
 */
enum class CoreType {
  ARC,
  DRAM,
  ACTIVE_ETH,
  IDLE_ETH,
  PCIE,
  TENSIX,
  ROUTER_ONLY,
  // TODO: this keeps compatibility with existing code in SocDescriptor
  // but it won't be needed later on
  HARVESTED,
  ETH,
};

 struct CoreCoord : public tt_xy_pair {
    CoreCoord() {}
    CoreCoord(const size_t x, const size_t y, const CoreType type, const CoordSystem coord_system)
        : tt_xy_pair(x, y), core_type(type), coord_system(coord_system) {}

    CoreType core_type;
    CoordSystem coord_system;
 };