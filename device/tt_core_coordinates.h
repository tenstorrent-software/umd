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
 * This is used both for V1 and V2.
 */
enum class CoordSystem {
    LOGICAL,
    PHYSICAL,
    VIRTUAL,
    TRANSLATED,
};

// ************************************************************************************************
// V1

/*
 * CoreType is an enum class that represents all types of cores
 * present on the Tenstorrent chip.
 */
enum class CoreType {
  ARC,
  DRAM,
  ETH,
  PCIE,
  TENSIX,
  ROUTER_ONLY,
};

 struct CoreCoord_V1 : public tt_xy_pair {
    CoreType core_type;
    CoordSystem coord_system;
 };

// ************************************************************************************************
// V2
struct CoreCoord_V2 : public tt_xy_pair {
    CoordSystem coord_system;
};

struct TensixCoreCoord_V2 : public CoreCoord_V2 {

};
 
struct DramCoreCoord_V2 : public CoreCoord_V2 {
    
};