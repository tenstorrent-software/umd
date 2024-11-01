// SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0
#include <thread>
#include <memory>
#include <random>

#include "device/tt_device.h"

TEST(CoreCoordinates, CoreCoordinatesReadWriteV1) {

    tt_SiliconDevice device = tt_SiliconDevice();

    device.start_device();

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> readback_vec = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    const uint32_t address = 0;
    const uint32_t chip = 0;

    tt_xy_pair worker_grid_size = device.get_worker_grid_size();

    // Write to tensix cores.
    for (size_t x = 0; x < worker_grid_size.x; x++) {
        for (size_t y = 0; y < worker_grid_size.y; y++) {
            
            // Construct the core coordinates.
            CoreCoord_V1 tensix_core_coords;
            tensix_core_coords.x = x;
            tensix_core_coords.y = y;
            tensix_core_coords.coord_system = CoordSystem::LOGICAL;
            // This step is not needed for V2 coordinates.
            tensix_core_coords.core_type = CoreType::TENSIX;
            
            device.write_to_device(vector_to_write.data(), vector_to_write.size() * sizeof(std::uint32_t), chip, tensix_core_coords, address);

            device_read_from_device(readback_vec.data(), readback_vec.size() * sizeof(std::uint32_t), chip, tensix_core_coords, address);

            EXPECT_EQ(vector_to_write, readback_vec);
        }
    }

    // Write to DRAM cores.
    const uint32_t num_channels = 6;
    const uint32_t num_subchannels = 2;
    
    for (std::uint32_t channel = 0; channel < num_channels; channel++) {
        for (std::uint32_t subchannel = 0; subchannel < num_subchannels; subchannel++) {
            CoreCoord_V1 dram_core_coords;
            dram_core_coords.x = channel;
            dram_core_coords.y = subchannel;
            dram_core_coords.coord_system = CoordSystem::LOGICAL;
            // This step is not needed for V2 coordinates.
            dram_core_coords.core_type = CoreType::DRAM;

            device.write_to_device(vector_to_write.data(), vector_to_write.size() * sizeof(std::uint32_t), chip, dram_core_coords, address);

            device_read_from_device(readback_vec.data(), readback_vec.size() * sizeof(std::uint32_t), chip, dram_core_coords, address);

            EXPECT_EQ(vector_to_write, readback_vec);
        }
    }

    device.close_device();
}

TEST(CoreCoordinates, CoreCoordinatesReadWriteV1) {

    tt_SiliconDevice device = tt_SiliconDevice();

    device.start_device();

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> readback_vec = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    const uint32_t address = 0;
    const uint32_t chip = 0;

    tt_xy_pair worker_grid_size = device.get_worker_grid_size();

    // Write to tensix cores.
    for (size_t x = 0; x < worker_grid_size.x; x++) {
        for (size_t y = 0; y < worker_grid_size.y; y++) {
            
            TensixCoreCoord_V2 tensix_core_coords;
            tensix_core_coords.x = x;
            tensix_core_coords.y = y;
            tensix_core_coords.coord_system = CoordSystem::LOGICAL;
            
            device.write_to_device(vector_to_write.data(), vector_to_write.size() * sizeof(std::uint32_t), chip, tensix_core_coords, address);

            device_read_from_device(readback_vec.data(), readback_vec.size() * sizeof(std::uint32_t), chip, tensix_core_coords, address);

            EXPECT_EQ(vector_to_write, readback_vec);
        }
    }

    // Write to DRAM cores.
    const uint32_t num_channels = 6;
    const uint32_t num_subchannels = 2;
    
    for (std::uint32_t channel = 0; channel < num_channels; channel++) {
        for (std::uint32_t subchannel = 0; subchannel < num_subchannels; subchannel++) {
            DramCoreCoord_V2 dram_core_coords;
            dram_core_coords.x = channel;
            dram_core_coords.y = subchannel;
            dram_core_coords.coord_system = CoordSystem::LOGICAL;

            device.write_to_device(vector_to_write.data(), vector_to_write.size() * sizeof(std::uint32_t), chip, dram_core_coords, address);

            device_read_from_device(readback_vec.data(), readback_vec.size() * sizeof(std::uint32_t), chip, dram_core_coords, address);

            EXPECT_EQ(vector_to_write, readback_vec);
        }
    }

    device.close_device();
}
