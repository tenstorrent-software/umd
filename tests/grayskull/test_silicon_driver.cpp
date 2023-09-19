#include "gtest/gtest.h"
#include <tt_device.h>
#include <device/tt_soc_descriptor.h>
#include "device_data.hpp"
#include "l1_address_map.h"
#include <thread>

TEST(SiliconDriverGS, Harvesting) {
    setenv("TT_BACKEND_HARVESTED_ROWS", "6,12", 1);
    std::set<chip_id_t> target_devices = {0, 1};
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {}; // Don't set any dynamic TLBs in this test
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/grayskull_10x12.yaml", "", target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);
    device.clean_system_resources();
    auto sdesc_per_chip = device.get_virtual_soc_descriptors();

    ASSERT_EQ(device.using_harvested_soc_descriptors(), true) << "Expected Driver to have performed harvesting";
    for(const auto& chip : sdesc_per_chip) {
        ASSERT_EQ(chip.second.workers.size(), 96) << "Expected SOC descriptor with harvesting to have 96 workers for chip " << chip.first;
    }
    ASSERT_EQ(device.get_harvesting_masks_for_soc_descriptors().at(0), 6) << "Expected first chip to have harvesting mask of 6";
    ASSERT_EQ(device.get_harvesting_masks_for_soc_descriptors().at(1), 12) << "Expected second chip to have harvesting mask of 12";
    device.close_device();
    unsetenv("TT_BACKEND_HARVESTED_ROWS");
}

TEST(SiliconDriverGS, CustomSocDesc) {
    setenv("TT_BACKEND_HARVESTED_ROWS", "6,12", 1);
    std::set<chip_id_t> target_devices = {0, 1};
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {}; // Don't set any dynamic TLBs in this test
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    // Initialize the driver with a 1x1 descriptor and explictly do not perform harvesting
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/grayskull_1x1_arch.yaml", "", target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config, false, false);
    device.clean_system_resources();
    auto sdesc_per_chip = device.get_virtual_soc_descriptors();
    ASSERT_EQ(device.using_harvested_soc_descriptors(), false) << "SOC descriptors should not be modified when harvesting is disabled";
    for(const auto& chip : sdesc_per_chip) {
        ASSERT_EQ(chip.second.workers.size(), 1) << "Expected 1x1 SOC descriptor to be unmodified by driver";
    }
    unsetenv("TT_BACKEND_HARVESTED_ROWS");
}

TEST(SiliconDriverGS, HarvestingRuntime) {
    setenv("TT_BACKEND_HARVESTED_ROWS", "6,12", 1);

    auto get_static_tlb_index = [] (tt_xy_pair target) {
        int flat_index = target.y * DEVICE_DATA.GRID_SIZE_X + target.x;
        if (flat_index == 0) {
            return -1;
        }
        return flat_index;
    };

    std::set<chip_id_t> target_devices = {0, 1};
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {{"SMALL_READ_WRITE_TLB", 157}}; // Use both static and dynamic TLBs here
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/grayskull_10x12.yaml", "", target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);


    for(int i = 0; i < target_devices.size(); i++) {
        // Iterate over devices and only setup static TLBs for functional worker cores
        auto& sdesc = device.get_virtual_soc_descriptors().at(i);
        for(auto& core : sdesc.workers) {
            // Statically mapping a 1MB TLB to this core, starting from address DATA_BUFFER_SPACE_BASE.
            device.configure_tlb(i, core, get_static_tlb_index(core), l1_mem::address_map::DATA_BUFFER_SPACE_BASE);
        }
    }

    device.setup_core_to_tlb_map(get_static_tlb_index);

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
    float timeout_in_seconds = 10;
    // Check functionality of Static TLBs by reading adn writing from statically mapped address space
    for(int i = 0; i < target_devices.size(); i++) {
        std::uint32_t address = l1_mem::address_map::DATA_BUFFER_SPACE_BASE;
        std::uint32_t dynamic_write_address = 0x30000000;
        for(int loop = 0; loop < 100; loop++){ // Write to each core a 100 times at different statically mapped addresses
            for(auto& core :  device.get_virtual_soc_descriptors().at(i).workers) {
                device.write_to_device(vector_to_write, tt_cxy_pair(i, core), address, "");
                device.write_to_device(vector_to_write, tt_cxy_pair(i, core), dynamic_write_address, "SMALL_READ_WRITE_TLB");
                auto start_time = std::chrono::high_resolution_clock::now();
                while(!(vector_to_write == readback_vec)) {
                    float wait_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count();
                    if(wait_duration > timeout_in_seconds) {
                        break;
                    }
                    device.read_from_device(readback_vec, tt_cxy_pair(i, core), address, 40, "");
                    device.read_from_device(dynamic_readback_vec, tt_cxy_pair(i, core), dynamic_write_address, 40, "SMALL_READ_WRITE_TLB");
                }
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                device.write_to_device(zeros, tt_cxy_pair(i, core), address, "SMALL_READ_WRITE_TLB"); // Clear any written data
                device.write_to_device(zeros, tt_cxy_pair(i, core), dynamic_write_address, "SMALL_READ_WRITE_TLB"); // Clear any written data
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

TEST(SiliconDriverGS, StaticTLB_RW) {
    auto get_static_tlb_index = [] (tt_xy_pair target) {
        int flat_index = target.y * DEVICE_DATA.GRID_SIZE_X + target.x;
        if (flat_index == 0) {
            return -1;
        }
        return flat_index;
    };
    std::set<chip_id_t> target_devices = {0, 1};

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {}; // Don't set any dynamic TLBs in this test
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/grayskull_10x12.yaml", "", target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);
    for(int i = 0; i < target_devices.size(); i++) {
        // Iterate over devices and only setup static TLBs for worker cores
        auto& sdesc = device.get_virtual_soc_descriptors().at(i);
        for(auto& core : sdesc.workers) {
            // Statically mapping a 1MB TLB to this core, starting from address DATA_BUFFER_SPACE_BASE.
            device.configure_tlb(i, core, get_static_tlb_index(core), l1_mem::address_map::DATA_BUFFER_SPACE_BASE);
        }
    }

    device.setup_core_to_tlb_map(get_static_tlb_index);

    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> readback_vec = {};
    std::vector<uint32_t> zeros = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    float timeout_in_seconds = 10;
    // Check functionality of Static TLBs by reading adn writing from statically mapped address space
    for(int i = 0; i < target_devices.size(); i++) {
        std::uint32_t address = l1_mem::address_map::DATA_BUFFER_SPACE_BASE;
        for(int loop = 0; loop < 100; loop++){ // Write to each core a 100 times at different statically mapped addresses
            for(auto& core :  device.get_virtual_soc_descriptors().at(i).workers) {
                device.write_to_device(vector_to_write, tt_cxy_pair(i, core), address, "");
                auto start_time = std::chrono::high_resolution_clock::now();
                while(!(vector_to_write == readback_vec)) {
                    float wait_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count();
                    if(wait_duration > timeout_in_seconds) {
                        break;
                    }
                    device.read_from_device(readback_vec, tt_cxy_pair(i, core), address, 40, "");
                }
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                device.write_to_device(zeros, tt_cxy_pair(i, core), address, "SMALL_READ_WRITE_TLB"); // Clear any written data
                readback_vec = {};
            }
            address += 0x20; // Increment by uint32_t size for each write
        }
    }
    device.close_device();
}

TEST(SiliconDriverGS, DynamicTLB_RW) {
    // Don't use any static TLBs in this test. All writes go through a dynamic TLB that needs to be reconfigured for each transaction
    std::set<chip_id_t> target_devices = {0, 1};

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {};
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    dynamic_tlb_config.insert({"SMALL_READ_WRITE_TLB", 157}); // Use this for all reads and writes to worker cores
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/grayskull_10x12.yaml", "", target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);

    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> zeros = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<uint32_t> readback_vec = {};
    float timeout_in_seconds = 10;

    for(int i = 0; i < target_devices.size(); i++) {
        std::uint32_t address = l1_mem::address_map::DATA_BUFFER_SPACE_BASE;
        for(int loop = 0; loop < 100; loop++){ // Write to each core a 100 times at different statically mapped addresses
            for(auto& core : device.get_virtual_soc_descriptors().at(i).workers) {
                device.write_to_device(vector_to_write, tt_cxy_pair(i, core), address, "SMALL_READ_WRITE_TLB");
                auto start_time = std::chrono::high_resolution_clock::now();
                while(!(vector_to_write == readback_vec)) {
                    float wait_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count();
                    if(wait_duration > timeout_in_seconds) {
                        break;
                    }
                    device.read_from_device(readback_vec, tt_cxy_pair(i, core), address, 40, "SMALL_READ_WRITE_TLB");
                }

                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                device.write_to_device(zeros, tt_cxy_pair(i, core), address, "SMALL_READ_WRITE_TLB"); // Clear any written data
                readback_vec = {};
            }
            address += 0x20; // Increment by uint32_t size for each write
        }
    }
    device.close_device();
}

TEST(SiliconDriverGS, MultiThreadedDevice) {
    // Have 2 threads read and write from a single device concurrently
    // All transactions go through a single Dynamic TLB. We want to make sure this is thread/process safe

    std::set<chip_id_t> target_devices = {0};

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {};
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    dynamic_tlb_config.insert({"SMALL_READ_WRITE_TLB", 157}); // Use this for all reads and writes to worker cores
    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/grayskull_10x12.yaml", "", target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);

    tt_device_params default_params;
    device.start_device(default_params);
    device.clean_system_resources();

    for(int i = 0; i < target_devices.size(); i++) {
        device.deassert_risc_reset(i);
    }

    std::thread th1 = std::thread([&] {
        std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        std::vector<uint32_t> readback_vec = {};
        float timeout_in_seconds = 10;
        std::uint32_t address = l1_mem::address_map::DATA_BUFFER_SPACE_BASE;
        for(int loop = 0; loop < 100; loop++) {
            for(auto& core : device.get_virtual_soc_descriptors().at(0).workers) {
                device.write_to_device(vector_to_write, tt_cxy_pair(0, core), address, "SMALL_READ_WRITE_TLB");
                auto start_time = std::chrono::high_resolution_clock::now();
                while(!(vector_to_write == readback_vec)) {
                    float wait_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count();
                    if(wait_duration > timeout_in_seconds) {
                        break;
                    }
                    device.read_from_device(readback_vec, tt_cxy_pair(0, core), address, 40, "SMALL_READ_WRITE_TLB");
                }
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                readback_vec = {};
            }
            address += 0x20;
        }
    });

    std::thread th2 = std::thread([&] {
        std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        std::vector<uint32_t> readback_vec = {};
        float timeout_in_seconds = 10;
        std::uint32_t address = 0x30000000;
        for(auto& core_ls : device.get_virtual_soc_descriptors().at(0).dram_cores) {
            for(int loop = 0; loop < 100; loop++) {
                for(auto& core : core_ls) {
                    device.write_to_device(vector_to_write, tt_cxy_pair(0, core), address, "SMALL_READ_WRITE_TLB");
                    auto start_time = std::chrono::high_resolution_clock::now();
                    while(!(vector_to_write == readback_vec)) {
                        float wait_duration = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - start_time).count();
                        if(wait_duration > timeout_in_seconds) {
                            break;
                    }
                    device.read_from_device(readback_vec, tt_cxy_pair(0, core), address, 40, "SMALL_READ_WRITE_TLB");
                }
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

TEST(SiliconDriverGS, LuwenIntegrationTest) {
    std::cout << "Running Luwen Integration Test" << std::endl;
    std::set<chip_id_t> target_devices = {0};

    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config = {};
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    dynamic_tlb_config.insert({"SMALL_READ_WRITE_TLB", 157}); // Use this for all reads and writes to worker cores

    tt_SiliconDevice device = tt_SiliconDevice("./tests/soc_descs/grayskull_10x12.yaml", "", target_devices, num_host_mem_ch_per_mmio_device, dynamic_tlb_config);

    device.close_device();
}
