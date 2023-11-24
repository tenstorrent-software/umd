// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "gtest/gtest.h"
#include <tt_device.h>
#include "device_data.hpp"
#include "eth_l1_address_map.h"
#include "l1_address_map.h"
#include "eth_interface.h"
#include "host_mem_address_map.h"
#include <thread>
#include <util.hpp>
#include <memory>

#include "device/tt_cluster_descriptor.h"

void set_params_for_remote_txn(tt_SiliconDevice& device) {
    // Populate address map and NOC parameters that the driver needs for remote transactions
    device.set_driver_host_address_params({host_mem::address_map::ETH_ROUTING_BLOCK_SIZE, host_mem::address_map::ETH_ROUTING_BUFFERS_START});

    device.set_driver_eth_interface_params({NOC_ADDR_LOCAL_BITS, NOC_ADDR_NODE_ID_BITS, ETH_RACK_COORD_WIDTH, CMD_BUF_SIZE_MASK, MAX_BLOCK_SIZE,
                                            REQUEST_CMD_QUEUE_BASE, RESPONSE_CMD_QUEUE_BASE, CMD_COUNTERS_SIZE_BYTES, REMOTE_UPDATE_PTR_SIZE_BYTES,
                                            CMD_DATA_BLOCK, CMD_WR_REQ, CMD_WR_ACK, CMD_RD_REQ, CMD_RD_DATA, CMD_BUF_SIZE, CMD_DATA_BLOCK_DRAM, ETH_ROUTING_DATA_BUFFER_ADDR,
                                            REQUEST_ROUTING_CMD_QUEUE_BASE, RESPONSE_ROUTING_CMD_QUEUE_BASE, CMD_BUF_PTR_MASK});
    
    device.set_device_l1_address_params({l1_mem::address_map::NCRISC_FIRMWARE_BASE, l1_mem::address_map::FIRMWARE_BASE,
                                  l1_mem::address_map::TRISC0_SIZE, l1_mem::address_map::TRISC1_SIZE, l1_mem::address_map::TRISC2_SIZE,
                                  l1_mem::address_map::TRISC_BASE, l1_mem::address_map::L1_BARRIER_BASE, eth_l1_mem::address_map::ERISC_BARRIER_BASE,
                                  eth_l1_mem::address_map::FW_VERSION_ADDR});
}

std::int32_t get_static_tlb_index(tt_xy_pair target) {
    bool is_eth_location = std::find(std::cbegin(DEVICE_DATA.ETH_LOCATIONS), std::cend(DEVICE_DATA.ETH_LOCATIONS), target) != std::cend(DEVICE_DATA.ETH_LOCATIONS);
    bool is_tensix_location = std::find(std::cbegin(DEVICE_DATA.T6_X_LOCATIONS), std::cend(DEVICE_DATA.T6_X_LOCATIONS), target.x) != std::cend(DEVICE_DATA.T6_X_LOCATIONS) &&
                            std::find(std::cbegin(DEVICE_DATA.T6_Y_LOCATIONS), std::cend(DEVICE_DATA.T6_Y_LOCATIONS), target.y) != std::cend(DEVICE_DATA.T6_Y_LOCATIONS);
    if (is_eth_location) {
        if (target.y == 6) {
            target.y = 1;
        }

        if (target.x >= 5) {
            target.x -= 1;
        }
        target.x -= 1;

        int flat_index = target.y * 8 + target.x;
        int tlb_index = flat_index;
        return tlb_index;

    } else if (is_tensix_location) {
        if (target.x >= 5) {
            target.x -= 1;
        }
        target.x -= 1;

        if (target.y >= 6) {
            target.y -= 1;
        }
        target.y -= 1;

        int flat_index = target.y * 8 + target.x;

        // All 80 get single 1MB TLB.
        int tlb_index = DEVICE_DATA.ETH_LOCATIONS.size() + flat_index;

        return tlb_index;
    } else {
        return -1;
    }
}

TEST(SiliconDriverWH, CreateDestroy) {
    std::set<chip_id_t> target_devices = {0, 1};
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {}; // Don't set any dynamic TLBs in this test
    tt_device_params default_params;
    // Initialize the driver with a 1x1 descriptor and explictly do not perform harvesting
    for(int i = 0; i < 1000; i++) {
        tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_1x1.yaml", GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config, false, false);
        set_params_for_remote_txn(device);
        device.start_device(default_params);
        device.clean_system_resources();
        for(int i = 0; i < target_devices.size(); i++) {
            device.deassert_risc_reset(i);
        }
        device.close_device();
    }

}

TEST(SiliconDriverWH, Harvesting) {
    setenv("TT_BACKEND_HARVESTED_ROWS", "30,60", 1);
    std::set<chip_id_t> target_devices = {0, 1};
    
    {
        std::unique_ptr<tt_ClusterDescriptor> cluster_desc_uniq = tt_ClusterDescriptor::create_from_yaml(GetClusterDescYAML().string());
        if (cluster_desc_uniq->get_number_of_chips() != target_devices.size()) {
            unsetenv("TT_BACKEND_HARVESTED_ROWS");
            GTEST_SKIP() << "SiliconDriverWH.Harvesting skipped because it can only be run on a two chip nebula system";
        }
    }

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {}; // Don't set any dynamic TLBs in this test
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_8x10.yaml", GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);
    device.clean_system_resources();
    auto sdesc_per_chip = device.get_virtual_soc_descriptors();

    ASSERT_EQ(device.using_harvested_soc_descriptors(), true) << "Expected Driver to have performed harvesting";

    for(const auto& chip : sdesc_per_chip) {
        ASSERT_EQ(chip.second.workers.size(), 48) << "Expected SOC descriptor with harvesting to have 48 workers for chip" << chip.first;
    }
    ASSERT_EQ(device.get_harvesting_masks_for_soc_descriptors().at(0), 30) << "Expected first chip to have harvesting mask of 30";
    ASSERT_EQ(device.get_harvesting_masks_for_soc_descriptors().at(1), 60) << "Expected second chip to have harvesting mask of 60";
    unsetenv("TT_BACKEND_HARVESTED_ROWS");
}

TEST(SiliconDriverWH, CustomSocDesc) {
    setenv("TT_BACKEND_HARVESTED_ROWS", "30,60", 1);
    std::set<chip_id_t> target_devices = {0, 1};

    {
        std::unique_ptr<tt_ClusterDescriptor> cluster_desc_uniq = tt_ClusterDescriptor::create_from_yaml(GetClusterDescYAML().string());
        if (cluster_desc_uniq->get_number_of_chips() != target_devices.size()) {
            unsetenv("TT_BACKEND_HARVESTED_ROWS");
            GTEST_SKIP() << "SiliconDriverWH.Harvesting skipped because it can only be run on a two chip nebula system";
        }
    }

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {}; // Don't set any dynamic TLBs in this test
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    // Initialize the driver with a 1x1 descriptor and explictly do not perform harvesting
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_1x1.yaml", GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config, false, false);
    device.clean_system_resources();
    auto sdesc_per_chip = device.get_virtual_soc_descriptors();
    
    ASSERT_EQ(device.using_harvested_soc_descriptors(), false) << "SOC descriptors should not be modified when harvesting is disabled";
    for(const auto& chip : sdesc_per_chip) {
        ASSERT_EQ(chip.second.workers.size(), 1) << "Expected 1x1 SOC descriptor to be unmodified by driver";
    }
    unsetenv("TT_BACKEND_HARVESTED_ROWS");
}

TEST(SiliconDriverWH, HarvestingRuntime) {
    setenv("TT_BACKEND_HARVESTED_ROWS", "30,60", 1);

    auto get_static_tlb_index_callback = [] (tt_xy_pair target) {
        return get_static_tlb_index(target);
    };

    std::set<chip_id_t> target_devices = {0, 1};

    {
        std::unique_ptr<tt_ClusterDescriptor> cluster_desc_uniq = tt_ClusterDescriptor::create_from_yaml(GetClusterDescYAML().string());
        if (cluster_desc_uniq->get_number_of_chips() != target_devices.size()) {
            unsetenv("TT_BACKEND_HARVESTED_ROWS");
            GTEST_SKIP() << "SiliconDriverWH.Harvesting skipped because it can only be run on a two chip nebula system";
        }
    }

    uint32_t num_host_mem_ch_per_mmio_device = 1;
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {{"SMALL_READ_WRITE_TLB", 157}}; // Use both static and dynamic TLBs here
    
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_8x10.yaml", GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);
    set_params_for_remote_txn(device);
    auto mmio_devices = device.get_target_mmio_device_ids();
    
    for(int i = 0; i < target_devices.size(); i++) {
        // Iterate over MMIO devices and only setup static TLBs for worker cores
        if(std::find(mmio_devices.begin(), mmio_devices.end(), i) != mmio_devices.end()) {
            auto& sdesc = device.get_virtual_soc_descriptors().at(i);
            for(auto& core : sdesc.workers) {
                // Statically mapping a 1MB TLB to this core, starting from address NCRISC_FIRMWARE_BASE.  
                device.configure_tlb(i, core, get_static_tlb_index_callback(core), l1_mem::address_map::NCRISC_FIRMWARE_BASE);
            }
        } 
    }
    device.setup_core_to_tlb_map(get_static_tlb_index_callback);
    
    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> dynamic_tlb_vector_to_write = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    std::vector<uint32_t> dynamic_readback_vec = {};
    std::vector<uint32_t> readback_vec = {};
    std::vector<uint32_t> zeros = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};


    for(int i = 0; i < target_devices.size(); i++) {
        std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
        std::uint32_t dynamic_write_address = 0x40000000;
        for(int loop = 0; loop < 100; loop++){ // Write to each core a 100 times at different statically mapped addresses
            for(auto& core : device.get_virtual_soc_descriptors().at(i).workers) {
                device.write_to_device(vector_to_write, tt_cxy_pair(i, core), address, "");
                device.write_to_device(vector_to_write, tt_cxy_pair(i, core), dynamic_write_address, "SMALL_READ_WRITE_TLB");
                device.wait_for_non_mmio_flush(); // Barrier to ensure that all writes over ethernet were commited
                device.read_from_device(readback_vec, tt_cxy_pair(i, core), address, 40, "");
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                device.wait_for_non_mmio_flush();
                device.write_to_device(zeros, tt_cxy_pair(i, core), address, "SMALL_READ_WRITE_TLB"); // Clear any written data
                device.read_from_device(dynamic_readback_vec, tt_cxy_pair(i, core), dynamic_write_address, 40, "SMALL_READ_WRITE_TLB");
                readback_vec = {};
                dynamic_readback_vec = {};
            }
            address += 0x20; // Increment by uint32_t size for each write
            dynamic_write_address += 0x20;
        }
    }
    device.close_device(); 
    unsetenv("TT_BACKEND_HARVESTED_ROWS");  
}

TEST(SiliconDriverWH, UnalignedStaticTLB_RW) {
    auto get_static_tlb_index_callback = [] (tt_xy_pair target) {
        return get_static_tlb_index(target);
    };

    std::set<chip_id_t> target_devices = {0, 1};

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {}; // Don't set any dynamic TLBs in this test
    dynamic_tlb_config["REG_TLB"] = 184;
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_8x10.yaml", GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);
    set_params_for_remote_txn(device);
    auto mmio_devices = device.get_target_mmio_device_ids();

    for(int i = 0; i < target_devices.size(); i++) {
        // Iterate over MMIO devices and only setup static TLBs for worker cores
        if(std::find(mmio_devices.begin(), mmio_devices.end(), i) != mmio_devices.end()) {
            auto& sdesc = device.get_virtual_soc_descriptors().at(i);
            for(auto& core : sdesc.workers) {
                // Statically mapping a 1MB TLB to this core, starting from address NCRISC_FIRMWARE_BASE.  
                device.configure_tlb(i, core, get_static_tlb_index_callback(core), l1_mem::address_map::NCRISC_FIRMWARE_BASE);
            }
        } 
    }

    device.setup_core_to_tlb_map(get_static_tlb_index_callback);
    
    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }

    std::vector<uint32_t> unaligned_sizes = {3, 14, 21, 255, 362, 430, 1022, 1023, 1025};
    for(int i = 0; i < 2; i++) {
        for(const auto& size : unaligned_sizes) {
            std::vector<uint8_t> write_vec(size, 0);
            for(int i = 0; i < size; i++){
                write_vec[i] = size + i;
            }
            std::vector<uint8_t> readback_vec(size, 0);
            std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
            for(int loop = 0; loop < 50; loop++){
                for(auto& core : device.get_virtual_soc_descriptors().at(i).workers) {
                    device.write_to_device(write_vec.data(), size, tt_cxy_pair(i, core), address, "");
                    device.read_from_device(readback_vec.data(), tt_cxy_pair(i, core), address, size, "");
                    ASSERT_EQ(readback_vec, write_vec);
                    readback_vec = std::vector<uint8_t>(size, 0);
                    device.write_to_sysmem(write_vec.data(), size, 0, 0, 0);
                    device.read_from_sysmem(readback_vec.data(), 0, 0, size, 0);
                    ASSERT_EQ(readback_vec, write_vec);
                    readback_vec = std::vector<uint8_t>(size, 0);
                }
                address += 0x20;
            }

        }
    }
    device.close_device();
}


TEST(SiliconDriverWH, StaticTLB_RW) {
    auto get_static_tlb_index_callback = [] (tt_xy_pair target) {
        return get_static_tlb_index(target);
    };

    std::set<chip_id_t> target_devices = {0, 1};

    {
        std::unique_ptr<tt_ClusterDescriptor> cluster_desc_uniq = tt_ClusterDescriptor::create_from_yaml(GetClusterDescYAML().string());
        if (cluster_desc_uniq->get_number_of_chips() != target_devices.size()) {
            unsetenv("TT_BACKEND_HARVESTED_ROWS");
            GTEST_SKIP() << "SiliconDriverWH.Harvesting skipped because it can only be run on a two chip nebula system";
        }
    }

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {}; // Don't set any dynamic TLBs in this test
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_8x10.yaml", GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);
    set_params_for_remote_txn(device);
    auto mmio_devices = device.get_target_mmio_device_ids();

    for(int i = 0; i < target_devices.size(); i++) {
        // Iterate over MMIO devices and only setup static TLBs for worker cores
        if(std::find(mmio_devices.begin(), mmio_devices.end(), i) != mmio_devices.end()) {
            auto& sdesc = device.get_virtual_soc_descriptors().at(i);
            for(auto& core : sdesc.workers) {
                // Statically mapping a 1MB TLB to this core, starting from address NCRISC_FIRMWARE_BASE.  
                device.configure_tlb(i, core, get_static_tlb_index_callback(core), l1_mem::address_map::NCRISC_FIRMWARE_BASE);
            }
        } 
    }

    device.setup_core_to_tlb_map(get_static_tlb_index_callback);
    
    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> readback_vec = {};
    std::vector<uint32_t> zeros = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Check functionality of Static TLBs by reading adn writing from statically mapped address space
    for(int i = 0; i < target_devices.size(); i++) {
        std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
        for(int loop = 0; loop < 100; loop++){ // Write to each core a 100 times at different statically mapped addresses
            for(auto& core : device.get_virtual_soc_descriptors().at(i).workers) {
                device.write_to_device(vector_to_write, tt_cxy_pair(i, core), address, "");
                device.wait_for_non_mmio_flush(); // Barrier to ensure that all writes over ethernet were commited
                device.read_from_device(readback_vec, tt_cxy_pair(i, core), address, 40, "");
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                device.wait_for_non_mmio_flush();
                device.write_to_device(zeros, tt_cxy_pair(i, core), address, "SMALL_READ_WRITE_TLB"); // Clear any written data
                readback_vec = {};
            }
            address += 0x20; // Increment by uint32_t size for each write
        }
    }
    device.close_device();    
}

TEST(SiliconDriverWH, DynamicTLB_RW) {
    // Don't use any static TLBs in this test. All writes go through a dynamic TLB that needs to be reconfigured for each transaction
    std::set<chip_id_t> target_devices = {0, 1};

    {
        std::unique_ptr<tt_ClusterDescriptor> cluster_desc_uniq = tt_ClusterDescriptor::create_from_yaml(GetClusterDescYAML().string());
        if (cluster_desc_uniq->get_number_of_chips() != target_devices.size()) {
            unsetenv("TT_BACKEND_HARVESTED_ROWS");
            GTEST_SKIP() << "SiliconDriverWH.Harvesting skipped because it can only be run on a two chip nebula system";
        }
    }

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {};
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    dynamic_tlb_config.insert({"SMALL_READ_WRITE_TLB", 157}); // Use this for all reads and writes to worker cores
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_8x10.yaml",  GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);

    set_params_for_remote_txn(device);

    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> zeros = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<uint32_t> readback_vec = {};

    for(int i = 0; i < target_devices.size(); i++) {
        std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
        for(int loop = 0; loop < 100; loop++){ // Write to each core a 100 times at different statically mapped addresses
            for(auto& core : device.get_virtual_soc_descriptors().at(i).workers) {
                device.write_to_device(vector_to_write, tt_cxy_pair(i, core), address, "SMALL_READ_WRITE_TLB");
                device.wait_for_non_mmio_flush(); // Barrier to ensure that all writes over ethernet were commited
                device.read_from_device(readback_vec, tt_cxy_pair(i, core), address, 40, "SMALL_READ_WRITE_TLB");
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                device.wait_for_non_mmio_flush();
                device.write_to_device(zeros, tt_cxy_pair(i, core), address, "SMALL_READ_WRITE_TLB");
                readback_vec = {};
            }
            address += 0x20; // Increment by uint32_t size for each write
        }
    }
    device.close_device();
}

TEST(SiliconDriverWH, MultiThreadedDevice) {
    // Have 2 threads read and write from a single device concurrently
    // All transactions go through a single Dynamic TLB. We want to make sure this is thread/process safe

    std::set<chip_id_t> target_devices = {0};

    {
        std::unique_ptr<tt_ClusterDescriptor> cluster_desc_uniq = tt_ClusterDescriptor::create_from_yaml(GetClusterDescYAML().string());
        if (cluster_desc_uniq->get_number_of_chips() > 2) {
            unsetenv("TT_BACKEND_HARVESTED_ROWS");
            GTEST_SKIP() << "SiliconDriverWH.Harvesting skipped because it can only be run on a one or two chip nebula system";
        }
    }

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {};
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    dynamic_tlb_config.insert({"SMALL_READ_WRITE_TLB", 157}); // Use this for all reads and writes to worker cores
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_8x10.yaml", GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);
    
    set_params_for_remote_txn(device);

    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }

    std::thread th1 = std::thread([&] {
        std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        std::vector<uint32_t> readback_vec = {};
        std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
        for(int loop = 0; loop < 100; loop++) {
            for(auto& core : device.get_virtual_soc_descriptors().at(0).workers) {
                device.write_to_device(vector_to_write, tt_cxy_pair(0, core), address, "SMALL_READ_WRITE_TLB");
                device.read_from_device(readback_vec, tt_cxy_pair(0, core), address, 40, "SMALL_READ_WRITE_TLB");
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                readback_vec = {};
            }
            address += 0x20;
        }
    });

    std::thread th2 = std::thread([&] {
        std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        std::vector<uint32_t> readback_vec = {};
        std::uint32_t address = 0x30000000;
        for(auto& core_ls : device.get_virtual_soc_descriptors().at(0).dram_cores) {
            for(int loop = 0; loop < 100; loop++) {
                for(auto& core : core_ls) {
                    device.write_to_device(vector_to_write, tt_cxy_pair(0, core), address, "SMALL_READ_WRITE_TLB");
                    device.read_from_device(readback_vec, tt_cxy_pair(0, core), address, 40, "SMALL_READ_WRITE_TLB");
                    ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                    readback_vec = {};
                }
                address += 0x20;
            }
        }
    });

    th1.join();
    th2.join();
    device.close_device();
}

TEST(SiliconDriverWH, MultiThreadedMemBar) {
    // Have 2 threads read and write from a single device concurrently
    // All (fairly large) transactions go through a static TLB. 
    // We want to make sure the memory barrier is thread/process safe.

    // Memory barrier flags get sent to address 0 for all channels in this test

    auto get_static_tlb_index_callback = [] (tt_xy_pair target) {
        return get_static_tlb_index(target);
    };

    std::set<chip_id_t> target_devices = {0};
    uint32_t base_addr = l1_mem::address_map::DATA_BUFFER_SPACE_BASE;
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {};
    dynamic_tlb_config.insert({"SMALL_READ_WRITE_TLB", 157}); // Use this for reading back membar values
    uint32_t num_host_mem_ch_per_mmio_device = 1;

    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/wormhole_b0_8x10.yaml", GetClusterDescYAML().string(), target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);
    set_params_for_remote_txn(device);
    for(int i = 0; i < target_devices.size(); i++) {
        // Iterate over devices and only setup static TLBs for functional worker cores
        auto& sdesc = device.get_virtual_soc_descriptors().at(i);
        for(auto& core : sdesc.workers) {
            // Statically mapping a 1MB TLB to this core, starting from address DATA_BUFFER_SPACE_BASE. 
            device.configure_tlb(i, core, get_static_tlb_index_callback(core), base_addr);
        }
    }
    device.setup_core_to_tlb_map(get_static_tlb_index_callback);

    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }
    std::vector<uint32_t> readback_membar_vec = {};
    for(auto& core : device.get_virtual_soc_descriptors().at(0).workers) {
        device.read_from_device(readback_membar_vec, tt_cxy_pair(0, core), l1_mem::address_map::L1_BARRIER_BASE, 4, "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(readback_membar_vec.at(0), 187); // Ensure that memory barriers were correctly initialized on all workers
        readback_membar_vec = {};
    }

    for(int chan = 0; chan <  device.get_virtual_soc_descriptors().at(0).get_num_dram_channels(); chan++) {
        auto core = device.get_virtual_soc_descriptors().at(0).get_core_for_dram_channel(chan, 0);
        device.read_from_device(readback_membar_vec, tt_cxy_pair(0, core), 0, 4, "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(readback_membar_vec.at(0), 187); // Ensure that memory barriers were correctly initialized on all DRAM
        readback_membar_vec = {};
    }
    
    for(auto& core : device.get_virtual_soc_descriptors().at(0).ethernet_cores) {
        device.read_from_device(readback_membar_vec, tt_cxy_pair(0, core), eth_l1_mem::address_map::ERISC_BARRIER_BASE, 4, "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(readback_membar_vec.at(0), 187); // Ensure that memory barriers were correctly initialized on all ethernet cores
        readback_membar_vec = {};
    }

    // Launch 2 thread accessing different locations of L1 and using memory barrier between write and read
    // Ensure now RAW race and membars are thread safe
    std::vector<uint32_t> vec1(2560);
    std::vector<uint32_t> vec2(2560);
    std::vector<uint32_t> zeros(2560, 0);

    for(int i = 0; i < vec1.size(); i++) {
        vec1.at(i) = i;
    }
    for(int i = 0; i < vec2.size(); i++) {
        vec2.at(i) = vec1.size() + i;
    }
    std::thread th1 = std::thread([&] {
        std::uint32_t address = base_addr;
        for(int loop = 0; loop < 50; loop++) {
            for(auto& core : device.get_virtual_soc_descriptors().at(0).workers) {
                std::vector<uint32_t> readback_vec = {};
                device.write_to_device(vec1, tt_cxy_pair(0, core), address, "");
                device.l1_membar(0, "SMALL_READ_WRITE_TLB", {core});
                device.read_from_device(readback_vec, tt_cxy_pair(0, core), address, 4*vec1.size(), "");
                ASSERT_EQ(readback_vec, vec1);
                device.write_to_device(zeros, tt_cxy_pair(0, core), address, "");
                readback_vec = {};
            }
            
        }
    });

    std::thread th2 = std::thread([&] {
        std::uint32_t address = base_addr + vec1.size() * 4;
        for(int loop = 0; loop < 50; loop++) {
            for(auto& core : device.get_virtual_soc_descriptors().at(0).workers) {
                std::vector<uint32_t> readback_vec = {};
                device.write_to_device(vec2, tt_cxy_pair(0, core), address, "");
                device.l1_membar(0, "SMALL_READ_WRITE_TLB", {core});
                device.read_from_device(readback_vec, tt_cxy_pair(0, core), address, 4*vec2.size(), "");
                ASSERT_EQ(readback_vec, vec2);
                device.write_to_device(zeros, tt_cxy_pair(0, core), address, "") ;
                readback_vec = {};
            }
        }
    });

    th1.join();
    th2.join();

    for(auto& core : device.get_virtual_soc_descriptors().at(0).workers) {
        device.read_from_device(readback_membar_vec, tt_cxy_pair(0, core), l1_mem::address_map::L1_BARRIER_BASE, 4, "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(readback_membar_vec.at(0), 187); // Ensure that memory barriers end up in the correct sate for workers
        readback_membar_vec = {};
    }

    for(auto& core : device.get_virtual_soc_descriptors().at(0).ethernet_cores) {
        device.read_from_device(readback_membar_vec, tt_cxy_pair(0, core), eth_l1_mem::address_map::ERISC_BARRIER_BASE, 4, "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(readback_membar_vec.at(0), 187); // Ensure that memory barriers end up in the correct sate for ethernet cores
        readback_membar_vec = {};
    }
    device.close_device();
}