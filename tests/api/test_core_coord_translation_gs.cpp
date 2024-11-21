/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "gtest/gtest.h"

#include "device/tt_soc_descriptor.h"
#include "tests/test_utils/generate_cluster_desc.hpp"
#include "tests/test_utils/soc_desc_test_utils.hpp"
#include "device/grayskull/grayskull_implementation.h"

// Grayskull workers - x-y annotation
// functional_workers:
//   [
//     1-1,  2-1,  3-1,  4-1,  5-1,  6-1,  7-1,  8-1,  9-1,  10-1,  11-1,  12-1,
//     1-2,  2-2,  3-2,  4-2,  5-2,  6-2,  7-2,  8-2,  9-2,  10-2,  11-2,  12-2,
//     1-3,  2-3,  3-3,  4-3,  5-3,  6-3,  7-3,  8-3,  9-3,  10-3,  11-3,  12-3,
//     1-4,  2-4,  3-4,  4-4,  5-4,  6-4,  7-4,  8-4,  9-4,  10-4,  11-4,  12-4,
//     1-5,  2-5,  3-5,  4-5,  5-5,  6-5,  7-5,  8-5,  9-5,  10-5,  11-5,  12-5,
//     1-7,  2-7,  3-7,  4-7,  5-7,  6-7,  7-7,  8-7,  9-7,  10-7,  11-7,  12-7,
//     1-8,  2-8,  3-8,  4-8,  5-8,  6-8,  7-8,  8-8,  9-8,  10-8,  11-8,  12-8,
//     1-9,  2-9,  3-9,  4-9,  5-9,  6-9,  7-9,  8-9,  9-9,  10-9,  11-9,  12-9,
//     1-10, 2-10, 3-10, 4-10, 5-10, 6-10, 7-10, 8-10, 9-10, 10-10, 11-10, 12-10,
//     1-11, 2-11, 3-11, 4-11, 5-11, 6-11, 7-11, 8-11, 9-11, 10-11, 11-11, 12-11
//   ]

// Tests that all physical coordinates are same as all virtual coordinates
// when there is no harvesting.
TEST(SocDescriptor, SocDescriptorGSNoHarvesting) {

    tt_SocDescriptor soc_desc = tt_SocDescriptor(test_utils::GetAbsPath("tests/soc_descs/grayskull_10x12.yaml"));

    // We expect full grid size since there is no harvesting.
    tt_xy_pair worker_grid_size = soc_desc.worker_grid_size;
    for (size_t x = 0; x < worker_grid_size.x; x++) {
        for (size_t y = 0; y < worker_grid_size.y; y++) {
            CoreCoord logical_coords = CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
            CoreCoord virtual_coords = soc_desc.to_virtual(logical_coords);
            CoreCoord physical_coords = soc_desc.to_physical(logical_coords);
            
            // Virtual and physical coordinates should be the same.
            EXPECT_EQ(physical_coords.x, virtual_coords.x);
            EXPECT_EQ(physical_coords.y, virtual_coords.y);
        }
    }
}

// // Test basic translation to virtual and physical noc coordinates.
// // We expect that the top left core will have virtual and physical coordinates (1, 1) and (1, 2) for
// // the logical coordinates if the first row is harvested.
TEST(SocDescriptor, SocDescriptorGSTopLeftCore) {

    tt_SocDescriptor soc_desc = tt_SocDescriptor(test_utils::GetAbsPath("tests/soc_descs/grayskull_10x12.yaml"));
    tt_xy_pair worker_grid_size = soc_desc.worker_grid_size;

    CoreCoord logical_coords = CoreCoord(0, 0, CoreType::TENSIX, CoordSystem::LOGICAL);

    // Always expect same virtual coordinate for (0, 0) logical coordinate.
    CoreCoord virtual_cords = soc_desc.to_virtual(logical_coords);
    EXPECT_EQ(virtual_cords, CoreCoord(1, 1, CoreType::TENSIX, CoordSystem::VIRTUAL));

    // This depends on harvesting mask. So expected physical coord is specific to this test and Wormhole arch.
    CoreCoord physical_cords = soc_desc.to_physical(logical_coords);
    EXPECT_EQ(physical_cords, CoreCoord(1, 1, CoreType::TENSIX, CoordSystem::PHYSICAL));
}

// // Test logical to physical, virtual and translated coordinates.
// // We always expect that physical, virtual and translated coordinates are the same.
TEST(SocDescriptor, SocDescriptorGSTranslatingCoords) {
    tt_SocDescriptor soc_desc = tt_SocDescriptor(test_utils::GetAbsPath("tests/soc_descs/grayskull_10x12.yaml"));
    tt_xy_pair worker_grid_size = soc_desc.worker_grid_size;

    for (size_t x = 0; x < worker_grid_size.x; x++) {
        for (size_t y = 0; y < worker_grid_size.y; y++) {
            CoreCoord logical_coords = CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
            CoreCoord virtual_coords = soc_desc.to_virtual(logical_coords);
            CoreCoord physical_coords = soc_desc.to_physical(logical_coords);
            CoreCoord translated_coords = soc_desc.to_translated(logical_coords);
            
            // Virtual, physical and translated coordinates should be the same.
            EXPECT_EQ(physical_coords.x, virtual_coords.x);
            EXPECT_EQ(physical_coords.y, virtual_coords.y);

            EXPECT_EQ(physical_coords.x, translated_coords.x);
            EXPECT_EQ(physical_coords.y, translated_coords.y);
        }
    }
}

// // Test logical to physical coordinate translation.
// // For the full grid of logical coordinates we expect that there are no duplicates of physical coordinates.
// // For the reverse mapping back of physical to logical coordinates we expect that same logical coordinates are returned as from original mapping.
TEST(SocDescriptor, SocDescriptorGSLogicalPhysicalMapping) {

    tt_SocDescriptor soc_desc = tt_SocDescriptor(test_utils::GetAbsPath("tests/soc_descs/grayskull_10x12.yaml"));

    std::map<CoreCoord, CoreCoord> logical_to_physical;
    std::set<CoreCoord> physical_coords_set;
    tt_xy_pair worker_grid_size = soc_desc.worker_grid_size;

    for (size_t x = 0; x < worker_grid_size.x; x++) {
        for (size_t y = 0; y < worker_grid_size.y; y++) {
            CoreCoord logical_coords = CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
            CoreCoord physical_coords = soc_desc.to_physical(logical_coords);
            logical_to_physical[logical_coords] = physical_coords;

            // Expect that logical to physical translation is 1-1 mapping. No duplicates for physical coordinates.
            EXPECT_EQ(physical_coords_set.count(physical_coords), 0);
            physical_coords_set.insert(physical_coords);
        }
    }

    EXPECT_EQ(physical_coords_set.size(), worker_grid_size.y * worker_grid_size.x);

    for (auto it : logical_to_physical) {
        CoreCoord physical_coords = it.second;
        CoreCoord logical_coords = soc_desc.to_logical(physical_coords);
        
        // Expect that reverse mapping of physical coordinates gives the same logical coordinates
        // using which we got the physical coordinates.
        EXPECT_EQ(it.first, logical_coords);
    }
}

// // Test logical to virtual coordinate translation.
// // For the full grid of logical coordinates we expect that there are no duplicates of virtual coordinates.
// // For the reverse mapping back of virtual to logical coordinates we expect that same logical coordinates are returned as from original mapping.
TEST(SocDescriptor, SocDescriptorGSLogicalVirtualMapping) {

    tt_SocDescriptor soc_desc = tt_SocDescriptor(test_utils::GetAbsPath("tests/soc_descs/grayskull_10x12.yaml"));

    std::map<CoreCoord, CoreCoord> logical_to_virtual;
    std::set<CoreCoord> virtual_coords_set;
    tt_xy_pair worker_grid_size = soc_desc.worker_grid_size;

    for (size_t x = 0; x < worker_grid_size.x; x++) {
        for (size_t y = 0; y < worker_grid_size.y; y++) {
            CoreCoord logical_coords = CoreCoord(x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
            CoreCoord virtual_coords = soc_desc.to_virtual(logical_coords);
            logical_to_virtual[logical_coords] = virtual_coords;

            // Expect that logical to virtual translation is 1-1 mapping. No duplicates for virtual coordinates.
            EXPECT_EQ(virtual_coords_set.count(virtual_coords), 0);
            virtual_coords_set.insert(virtual_coords);
        }
    }

    EXPECT_EQ(virtual_coords_set.size(), worker_grid_size.y * worker_grid_size.x);

    for (auto it : logical_to_virtual) {
        CoreCoord virtual_coords = it.second;
        CoreCoord logical_coords = soc_desc.to_logical(virtual_coords);

        // Expect that reverse mapping of virtual coordinates gives the same logical coordinates
        // using which we got the virtual coordinates.
        EXPECT_EQ(it.first, logical_coords);
    }
}

TEST(CoordinateManager, CoordinateManagerGSDRAMNoHarvesting) {
    tt_SocDescriptor soc_desc = tt_SocDescriptor(test_utils::GetAbsPath("tests/soc_descs/grayskull_10x12.yaml"), 0 ,0);

    const std::size_t num_dram_banks = tt::umd::grayskull::NUM_DRAM_BANKS;
    const std::vector<tt_xy_pair>& dram_cores = tt::umd::grayskull::DRAM_CORES;

    for (std::size_t dram_bank = 0; dram_bank < num_dram_banks; dram_bank++) {
        const CoreCoord dram_logical(dram_bank, 0, CoreType::DRAM, CoordSystem::LOGICAL);
        const CoreCoord expected_physical = CoreCoord(dram_cores[dram_bank].x, dram_cores[dram_bank].y, CoreType::DRAM, CoordSystem::PHYSICAL);

        const CoreCoord dram_physical = soc_desc.to_physical(dram_logical);

        EXPECT_EQ(dram_physical, expected_physical);
    }
}