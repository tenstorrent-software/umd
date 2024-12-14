// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include <umd/device/cluster.h>

#include <memory>
#include <thread>

#include "eth_l1_address_map.h"
#include "gtest/gtest.h"
#include "host_mem_address_map.h"
#include "l1_address_map.h"
#include "tests/test_utils/device_test_utils.hpp"
#include "tests/test_utils/generate_cluster_desc.hpp"
#include "umd/device/blackhole_implementation.h"
#include "umd/device/tt_cluster_descriptor.h"

using namespace tt::umd;

void set_params_for_remote_txn(Cluster& device) {
    // Populate address map and NOC parameters that the driver needs for remote transactions
    device.set_device_l1_address_params(
        {l1_mem::address_map::L1_BARRIER_BASE,
         eth_l1_mem::address_map::ERISC_BARRIER_BASE,
         eth_l1_mem::address_map::FW_VERSION_ADDR});
}

std::int32_t get_static_tlb_index(tt_xy_pair target) {
    bool is_eth_location =
        std::find(std::begin(tt::umd::blackhole::ETH_LOCATIONS), std::end(tt::umd::blackhole::ETH_LOCATIONS), target) !=
        std::end(tt::umd::blackhole::ETH_LOCATIONS);
    bool is_tensix_location =
        std::find(
            std::begin(tt::umd::blackhole::T6_X_LOCATIONS), std::end(tt::umd::blackhole::T6_X_LOCATIONS), target.x) !=
            std::end(tt::umd::blackhole::T6_X_LOCATIONS) &&
        std::find(
            std::begin(tt::umd::blackhole::T6_Y_LOCATIONS), std::end(tt::umd::blackhole::T6_Y_LOCATIONS), target.y) !=
            std::end(tt::umd::blackhole::T6_Y_LOCATIONS);
    if (is_eth_location) {
        if (target.y == 6) {
            target.y = 1;
        }

        if (target.x >= 5) {
            target.x -= 1;
        }
        target.x -= 1;

        int flat_index = target.y * 14 + target.x;
        int tlb_index = flat_index;
        return tlb_index;

    } else if (is_tensix_location) {
        if (target.x >= 8) {
            target.x -= 2;
        }
        target.x -= 1;  // First x index is 1

        target.y -= 2;  // First y index is 2

        int flat_index = target.y * 14 + target.x;

        // All 140 get single 2MB TLB.
        int tlb_index = tt::umd::blackhole::ETH_LOCATIONS.size() + flat_index;

        return tlb_index;
    } else {
        return -1;
    }
}

std::set<chip_id_t> get_target_devices() {
    std::set<chip_id_t> target_devices;
    std::unique_ptr<tt_ClusterDescriptor> cluster_desc_uniq = tt_ClusterDescriptor::create();
    for (int i = 0; i < cluster_desc_uniq->get_number_of_chips(); i++) {
        target_devices.insert(i);
    }
    return target_devices;
}



TEST(SiliconDriverBH, CreateDestroy) {
    std::set<chip_id_t> target_devices = get_target_devices();
    uint32_t num_host_mem_ch_per_mmio_device = 1;
    tt_device_params default_params;
    // Initialize the driver with a 1x1 descriptor and explictly do not perform harvesting
    for (int i = 0; i < 50; i++) {
        Cluster device = Cluster(
            test_utils::GetAbsPath("tests/soc_descs/blackhole_140_arch_no_eth.yaml"),
            target_devices,
            num_host_mem_ch_per_mmio_device,
            false,
            true,
            false);
        set_params_for_remote_txn(device);
        std::cout << "Starting device " << std::endl;
        device.start_device(default_params);
        // std::cout << "Deasserting risc reset " << std::endl;
        // device.deassert_risc_reset();
        std::cout << "Closing device " << std::endl;
        device.close_device();
    }
}

#if 0


TEST(SiliconDriverBH, UnalignedStaticTLB_RW) {
    auto get_static_tlb_index_callback = [](tt_xy_pair target) { return get_static_tlb_index(target); };

    std::set<chip_id_t> target_devices = get_target_devices();

    uint32_t num_host_mem_ch_per_mmio_device = 1;

    Cluster device = Cluster(num_host_mem_ch_per_mmio_device, false, true, true);
    set_params_for_remote_txn(device);
    auto mmio_devices = device.get_target_mmio_device_ids();

    for (int i = 0; i < target_devices.size(); i++) {
        // Iterate over MMIO devices and only setup static TLBs for worker cores
        if (std::find(mmio_devices.begin(), mmio_devices.end(), i) != mmio_devices.end()) {
            auto& sdesc = device.get_soc_descriptor(i);
            for (auto& core : sdesc.workers) {
                // Statically mapping a 1MB TLB to this core, starting from address NCRISC_FIRMWARE_BASE.
                device.configure_tlb(
                    i, core, get_static_tlb_index_callback(core), l1_mem::address_map::NCRISC_FIRMWARE_BASE);
            }
            device.setup_core_to_tlb_map(i, get_static_tlb_index_callback);
        }
    }

    tt_device_params default_params;
    device.start_device(default_params);
    device.deassert_risc_reset();

    std::vector<uint32_t> unaligned_sizes = {3, 14, 21, 255, 362, 430, 1022, 1023, 1025};
    for (int i = 0; i < target_devices.size(); i++) {
        for (const auto& size : unaligned_sizes) {
            std::vector<uint8_t> write_vec(size, 0);
            for (int i = 0; i < size; i++) {
                write_vec[i] = size + i;
            }
            std::vector<uint8_t> readback_vec(size, 0);
            std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
            for (int loop = 0; loop < 50; loop++) {
                for (auto& core : device.get_soc_descriptor(i).workers) {
                    device.write_to_device(write_vec.data(), size, tt_cxy_pair(i, core), address, "");
                    device.wait_for_non_mmio_flush();
                    device.read_from_device(readback_vec.data(), tt_cxy_pair(i, core), address, size, "");
                    ASSERT_EQ(readback_vec, write_vec);
                    readback_vec = std::vector<uint8_t>(size, 0);
                    device.write_to_sysmem(write_vec.data(), size, 0, 0, 0);
                    device.read_from_sysmem(readback_vec.data(), 0, 0, size, 0);
                    ASSERT_EQ(readback_vec, write_vec);
                    readback_vec = std::vector<uint8_t>(size, 0);
                    device.wait_for_non_mmio_flush();
                }
                address += 0x20;
            }
        }
    }
    device.close_device();
}

TEST(SiliconDriverBH, StaticTLB_RW) {
    auto get_static_tlb_index_callback = [](tt_xy_pair target) { return get_static_tlb_index(target); };

    std::set<chip_id_t> target_devices = get_target_devices();

    uint32_t num_host_mem_ch_per_mmio_device = 1;

    Cluster device = Cluster(num_host_mem_ch_per_mmio_device, false, true, true);
    set_params_for_remote_txn(device);
    auto mmio_devices = device.get_target_mmio_device_ids();

    for (int i = 0; i < target_devices.size(); i++) {
        // Iterate over MMIO devices and only setup static TLBs for worker cores
        if (std::find(mmio_devices.begin(), mmio_devices.end(), i) != mmio_devices.end()) {
            auto& sdesc = device.get_soc_descriptor(i);
            for (auto& core : sdesc.workers) {
                // Statically mapping a 2MB TLB to this core, starting from address NCRISC_FIRMWARE_BASE.
                device.configure_tlb(
                    i, core, get_static_tlb_index_callback(core), l1_mem::address_map::NCRISC_FIRMWARE_BASE);
            }
            device.setup_core_to_tlb_map(i, get_static_tlb_index_callback);
        }
    }

    printf("MT: Static TLBs set\n");

    tt_device_params default_params;
    device.start_device(default_params);
    device.deassert_risc_reset();

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> readback_vec = {};
    std::vector<uint32_t> zeros = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    // Check functionality of Static TLBs by reading adn writing from statically mapped address space
    for (int i = 0; i < target_devices.size(); i++) {
        std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
        for (int loop = 0; loop < 1;
             loop++) {  // Write to each core a 100 times at different statically mapped addresses
            for (auto& core : device.get_soc_descriptor(i).workers) {
                device.write_to_device(
                    vector_to_write.data(),
                    vector_to_write.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, core),
                    address,
                    "");
                device.wait_for_non_mmio_flush();  // Barrier to ensure that all writes over ethernet were commited
                test_utils::read_data_from_device(device, readback_vec, tt_cxy_pair(i, core), address, 40, "");
                ASSERT_EQ(vector_to_write, readback_vec)
                    << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                device.wait_for_non_mmio_flush();
                device.write_to_device(
                    zeros.data(),
                    zeros.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, core),
                    address,
                    "SMALL_READ_WRITE_TLB");  // Clear any written data
                device.wait_for_non_mmio_flush();
                readback_vec = {};
            }
            address += 0x20;  // Increment by uint32_t size for each write
        }
    }
    device.close_device();
}

TEST(SiliconDriverBH, DynamicTLB_RW) {
    // Don't use any static TLBs in this test. All writes go through a dynamic TLB that needs to be reconfigured for
    // each transaction
    std::set<chip_id_t> target_devices = get_target_devices();

    uint32_t num_host_mem_ch_per_mmio_device = 1;
    Cluster device = Cluster(num_host_mem_ch_per_mmio_device, false, true, true);

    set_params_for_remote_txn(device);

    tt_device_params default_params;
    device.start_device(default_params);
    // MT: Don't deassert risc resets since there's no loaded FW
    // device.deassert_risc_reset();

    std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
    std::vector<uint32_t> zeros = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    std::vector<uint32_t> readback_vec = {};

    for (int i = 0; i < target_devices.size(); i++) {
        std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
        for (int loop = 0; loop < 100;
             loop++) {  // Write to each core a 100 times at different statically mapped addresses
            for (auto& core : device.get_soc_descriptor(i).workers) {
                device.write_to_device(
                    vector_to_write.data(),
                    vector_to_write.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, core),
                    address,
                    "SMALL_READ_WRITE_TLB");
                device.wait_for_non_mmio_flush();  // Barrier to ensure that all writes over ethernet were commited
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(i, core), address, 40, "SMALL_READ_WRITE_TLB");
                ASSERT_EQ(vector_to_write, readback_vec)
                    << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                device.wait_for_non_mmio_flush();
                device.write_to_device(
                    zeros.data(),
                    zeros.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, core),
                    address,
                    "SMALL_READ_WRITE_TLB");
                device.wait_for_non_mmio_flush();
                readback_vec = {};
            }
            address += 0x20;  // Increment by uint32_t size for each write
        }
    }
    printf("Target Tensix cores completed\n");

    // Target DRAM channel 0
    constexpr int NUM_CHANNELS = 8;
    std::vector<uint32_t> dram_vector_to_write = {10, 11, 12, 13, 14, 15, 16, 17, 18, 19};
    std::uint32_t address = 0x400;
    for (int i = 0; i < target_devices.size(); i++) {
        for (int loop = 0; loop < 100;
             loop++) {  // Write to each core a 100 times at different statically mapped addresses
            for (int ch = 0; ch < NUM_CHANNELS; ch++) {
                std::vector<tt_xy_pair> chan = device.get_soc_descriptor(i).dram_cores.at(ch);
                tt_xy_pair subchan = chan.at(0);
                device.write_to_device(
                    vector_to_write.data(),
                    vector_to_write.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, subchan),
                    address,
                    "SMALL_READ_WRITE_TLB");
                device.wait_for_non_mmio_flush();  // Barrier to ensure that all writes over ethernet were commited
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(i, subchan), address, 40, "SMALL_READ_WRITE_TLB");
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << subchan.x << "-"
                                                         << subchan.y << "does not match what was written";
                device.wait_for_non_mmio_flush();
                device.write_to_device(
                    zeros.data(),
                    zeros.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, subchan),
                    address,
                    "SMALL_READ_WRITE_TLB");
                device.wait_for_non_mmio_flush();
                readback_vec = {};
                address += 0x20;  // Increment by uint32_t size for each write
            }
        }
    }
    printf("Target DRAM completed\n");

    device.close_device();
}

TEST(SiliconDriverBH, MultiThreadedDevice) {
    // Have 2 threads read and write from a single device concurrently
    // All transactions go through a single Dynamic TLB. We want to make sure this is thread/process safe

    std::set<chip_id_t> target_devices = get_target_devices();

    uint32_t num_host_mem_ch_per_mmio_device = 1;
    Cluster device = Cluster(num_host_mem_ch_per_mmio_device, false, true, true);

    set_params_for_remote_txn(device);

    tt_device_params default_params;
    device.start_device(default_params);
    device.deassert_risc_reset();

    std::thread th1 = std::thread([&] {
        std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        std::vector<uint32_t> readback_vec = {};
        std::uint32_t address = l1_mem::address_map::NCRISC_FIRMWARE_BASE;
        for (int loop = 0; loop < 100; loop++) {
            for (auto& core : device.get_soc_descriptor(0).workers) {
                device.write_to_device(
                    vector_to_write.data(),
                    vector_to_write.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(0, core),
                    address,
                    "SMALL_READ_WRITE_TLB");
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(0, core), address, 40, "SMALL_READ_WRITE_TLB");
                ASSERT_EQ(vector_to_write, readback_vec)
                    << "Vector read back from core " << core.x << "-" << core.y << "does not match what was written";
                readback_vec = {};
            }
            address += 0x20;
        }
    });

    std::thread th2 = std::thread([&] {
        std::vector<uint32_t> vector_to_write = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9};
        std::vector<uint32_t> readback_vec = {};
        std::uint32_t address = 0x30000000;
        for (auto& core_ls : device.get_soc_descriptor(0).dram_cores) {
            for (int loop = 0; loop < 100; loop++) {
                for (auto& core : core_ls) {
                    device.write_to_device(
                        vector_to_write.data(),
                        vector_to_write.size() * sizeof(std::uint32_t),
                        tt_cxy_pair(0, core),
                        address,
                        "SMALL_READ_WRITE_TLB");
                    test_utils::read_data_from_device(
                        device, readback_vec, tt_cxy_pair(0, core), address, 40, "SMALL_READ_WRITE_TLB");
                    ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y
                                                             << "does not match what was written";
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

TEST(SiliconDriverBH, MultiThreadedMemBar) {
    // Have 2 threads read and write from a single device concurrently
    // All (fairly large) transactions go through a static TLB.
    // We want to make sure the memory barrier is thread/process safe.

    // Memory barrier flags get sent to address 0 for all channels in this test
    auto get_static_tlb_index_callback = [](tt_xy_pair target) { return get_static_tlb_index(target); };

    std::set<chip_id_t> target_devices = get_target_devices();
    uint32_t base_addr = l1_mem::address_map::DATA_BUFFER_SPACE_BASE;
    uint32_t num_host_mem_ch_per_mmio_device = 1;

    Cluster device = Cluster(num_host_mem_ch_per_mmio_device, false, true, true);
    set_params_for_remote_txn(device);
    for (int i = 0; i < target_devices.size(); i++) {
        // Iterate over devices and only setup static TLBs for functional worker cores
        auto& sdesc = device.get_soc_descriptor(i);
        for (auto& core : sdesc.workers) {
            // Statically mapping a 1MB TLB to this core, starting from address DATA_BUFFER_SPACE_BASE.
            device.configure_tlb(i, core, get_static_tlb_index_callback(core), base_addr);
        }
        device.setup_core_to_tlb_map(i, get_static_tlb_index_callback);
    }

    tt_device_params default_params;
    device.start_device(default_params);
    device.deassert_risc_reset();

    std::vector<uint32_t> readback_membar_vec = {};
    for (auto& core : device.get_soc_descriptor(0).workers) {
        test_utils::read_data_from_device(
            device,
            readback_membar_vec,
            tt_cxy_pair(0, core),
            l1_mem::address_map::L1_BARRIER_BASE,
            4,
            "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(
            readback_membar_vec.at(0), 187);  // Ensure that memory barriers were correctly initialized on all workers
        readback_membar_vec = {};
    }

    for (int chan = 0; chan < device.get_soc_descriptor(0).get_num_dram_channels(); chan++) {
        auto core = device.get_soc_descriptor(0).get_core_for_dram_channel(chan, 0);
        test_utils::read_data_from_device(
            device, readback_membar_vec, tt_cxy_pair(0, core), 0, 4, "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(
            readback_membar_vec.at(0), 187);  // Ensure that memory barriers were correctly initialized on all DRAM
        readback_membar_vec = {};
    }

    for (auto& core : device.get_soc_descriptor(0).ethernet_cores) {
        test_utils::read_data_from_device(
            device,
            readback_membar_vec,
            tt_cxy_pair(0, core),
            eth_l1_mem::address_map::ERISC_BARRIER_BASE,
            4,
            "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(
            readback_membar_vec.at(0),
            187);  // Ensure that memory barriers were correctly initialized on all ethernet cores
        readback_membar_vec = {};
    }

    // Launch 2 thread accessing different locations of L1 and using memory barrier between write and read
    // Ensure now RAW race and membars are thread safe
    std::vector<uint32_t> vec1(2560);
    std::vector<uint32_t> vec2(2560);
    std::vector<uint32_t> zeros(2560, 0);

    for (int i = 0; i < vec1.size(); i++) {
        vec1.at(i) = i;
    }
    for (int i = 0; i < vec2.size(); i++) {
        vec2.at(i) = vec1.size() + i;
    }
    std::thread th1 = std::thread([&] {
        std::uint32_t address = base_addr;
        for (int loop = 0; loop < 50; loop++) {
            for (auto& core : device.get_soc_descriptor(0).workers) {
                std::vector<uint32_t> readback_vec = {};
                device.write_to_device(
                    vec1.data(), vec1.size() * sizeof(std::uint32_t), tt_cxy_pair(0, core), address, "");
                device.l1_membar(0, "SMALL_READ_WRITE_TLB", {core});
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(0, core), address, 4 * vec1.size(), "");
                ASSERT_EQ(readback_vec, vec1);
                device.write_to_device(
                    zeros.data(), zeros.size() * sizeof(std::uint32_t), tt_cxy_pair(0, core), address, "");
                readback_vec = {};
            }
        }
    });

    std::thread th2 = std::thread([&] {
        std::uint32_t address = base_addr + vec1.size() * 4;
        for (int loop = 0; loop < 50; loop++) {
            for (auto& core : device.get_soc_descriptor(0).workers) {
                std::vector<uint32_t> readback_vec = {};
                device.write_to_device(
                    vec2.data(), vec2.size() * sizeof(std::uint32_t), tt_cxy_pair(0, core), address, "");
                device.l1_membar(0, "SMALL_READ_WRITE_TLB", {core});
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(0, core), address, 4 * vec2.size(), "");
                ASSERT_EQ(readback_vec, vec2);
                device.write_to_device(
                    zeros.data(), zeros.size() * sizeof(std::uint32_t), tt_cxy_pair(0, core), address, "");
                readback_vec = {};
            }
        }
    });

    th1.join();
    th2.join();

    for (auto& core : device.get_soc_descriptor(0).workers) {
        test_utils::read_data_from_device(
            device,
            readback_membar_vec,
            tt_cxy_pair(0, core),
            l1_mem::address_map::L1_BARRIER_BASE,
            4,
            "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(
            readback_membar_vec.at(0), 187);  // Ensure that memory barriers end up in the correct sate for workers
        readback_membar_vec = {};
    }

    for (auto& core : device.get_soc_descriptor(0).ethernet_cores) {
        test_utils::read_data_from_device(
            device,
            readback_membar_vec,
            tt_cxy_pair(0, core),
            eth_l1_mem::address_map::ERISC_BARRIER_BASE,
            4,
            "SMALL_READ_WRITE_TLB");
        ASSERT_EQ(
            readback_membar_vec.at(0),
            187);  // Ensure that memory barriers end up in the correct sate for ethernet cores
        readback_membar_vec = {};
    }
    device.close_device();
}

TEST(SiliconDriverBH, DISABLED_BroadcastWrite) {  // Cannot broadcast to tensix/ethernet and DRAM simultaneously on
                                                  // Blackhole .. wait_for_non_mmio_flush() is not working as expected?
    // Broadcast multiple vectors to tensix and dram grid. Verify broadcasted data is read back correctly
    std::set<chip_id_t> target_devices = get_target_devices();

    uint32_t num_host_mem_ch_per_mmio_device = 1;

    Cluster device = Cluster(num_host_mem_ch_per_mmio_device, false, true, true);
    set_params_for_remote_txn(device);
    auto mmio_devices = device.get_target_mmio_device_ids();

    tt_device_params default_params;
    device.start_device(default_params);
    device.deassert_risc_reset();
    std::vector<uint32_t> broadcast_sizes = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384};
    uint32_t address = l1_mem::address_map::DATA_BUFFER_SPACE_BASE;
    std::set<uint32_t> rows_to_exclude = {0, 6};
    std::set<uint32_t> cols_to_exclude = {0, 5};
    std::set<uint32_t> rows_to_exclude_for_dram_broadcast = {};
    std::set<uint32_t> cols_to_exclude_for_dram_broadcast = {1, 2, 3, 4, 6, 7, 8, 9};

    for (const auto& size : broadcast_sizes) {
        std::vector<uint32_t> vector_to_write(size);
        std::vector<uint32_t> zeros(size);
        std::vector<uint32_t> readback_vec = {};
        for (int i = 0; i < size; i++) {
            vector_to_write[i] = i;
            zeros[i] = 0;
        }
        // Broadcast to Tensix
        device.broadcast_write_to_cluster(
            vector_to_write.data(),
            vector_to_write.size() * 4,
            address,
            {},
            rows_to_exclude,
            cols_to_exclude,
            "LARGE_WRITE_TLB");
        device.wait_for_non_mmio_flush();  // flush here so we don't simultaneously broadcast to DRAM?
        // Broadcast to DRAM
        device.broadcast_write_to_cluster(
            vector_to_write.data(),
            vector_to_write.size() * 4,
            address,
            {},
            rows_to_exclude_for_dram_broadcast,
            cols_to_exclude_for_dram_broadcast,
            "LARGE_WRITE_TLB");
        device.wait_for_non_mmio_flush();

        for (const auto i : target_devices) {
            for (const auto& core : device.get_soc_descriptor(i).workers) {
                if (rows_to_exclude.find(core.y) != rows_to_exclude.end()) {
                    continue;
                }
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(i, core), address, vector_to_write.size() * 4, "LARGE_READ_TLB");
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y
                                                         << "does not match what was broadcasted";
                device.write_to_device(
                    zeros.data(),
                    zeros.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, core),
                    address,
                    "LARGE_WRITE_TLB");  // Clear any written data
                readback_vec = {};
            }
            for (int chan = 0; chan < device.get_soc_descriptor(i).get_num_dram_channels(); chan++) {
                const auto& core = device.get_soc_descriptor(i).get_core_for_dram_channel(chan, 0);
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(i, core), address, vector_to_write.size() * 4, "LARGE_READ_TLB");
                ASSERT_EQ(vector_to_write, readback_vec)
                    << "Vector read back from DRAM core " << i << " " << core.x << "-" << core.y
                    << " does not match what was broadcasted " << size;
                device.write_to_device(
                    zeros.data(),
                    zeros.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, core),
                    address,
                    "LARGE_WRITE_TLB");  // Clear any written data
                readback_vec = {};
            }
        }
        // Wait for data to be cleared before writing next block
        device.wait_for_non_mmio_flush();
    }
    device.close_device();
}

TEST(SiliconDriverBH, DISABLED_VirtualCoordinateBroadcast) {  // same problem as above..
    // Broadcast multiple vectors to tensix and dram grid. Verify broadcasted data is read back correctly
    std::set<chip_id_t> target_devices = get_target_devices();

    uint32_t num_host_mem_ch_per_mmio_device = 1;

    Cluster device = Cluster(num_host_mem_ch_per_mmio_device, false, true, true);
    set_params_for_remote_txn(device);
    auto mmio_devices = device.get_target_mmio_device_ids();

    tt_device_params default_params;
    device.start_device(default_params);
    auto eth_version = device.get_ethernet_fw_version();
    bool virtual_bcast_supported =
        (eth_version >= tt_version(6, 8, 0) || eth_version == tt_version(6, 7, 241)) && device.translation_tables_en;
    if (!virtual_bcast_supported) {
        device.close_device();
        GTEST_SKIP() << "SiliconDriverWH.VirtualCoordinateBroadcast skipped since ethernet version does not support "
                        "Virtual Coordinate Broadcast or NOC translation is not enabled";
    }

    device.deassert_risc_reset();
    std::vector<uint32_t> broadcast_sizes = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384};
    uint32_t address = l1_mem::address_map::DATA_BUFFER_SPACE_BASE;
    std::set<uint32_t> rows_to_exclude = {0, 3, 5, 6, 8, 9};
    std::set<uint32_t> cols_to_exclude = {0, 5};
    std::set<uint32_t> rows_to_exclude_for_dram_broadcast = {};
    std::set<uint32_t> cols_to_exclude_for_dram_broadcast = {1, 2, 3, 4, 6, 7, 8, 9};

    for (const auto& size : broadcast_sizes) {
        std::vector<uint32_t> vector_to_write(size);
        std::vector<uint32_t> zeros(size);
        std::vector<uint32_t> readback_vec = {};
        for (int i = 0; i < size; i++) {
            vector_to_write[i] = i;
            zeros[i] = 0;
        }
        // Broadcast to Tensix
        device.broadcast_write_to_cluster(
            vector_to_write.data(),
            vector_to_write.size() * 4,
            address,
            {},
            rows_to_exclude,
            cols_to_exclude,
            "LARGE_WRITE_TLB");
        // Broadcast to DRAM
        device.broadcast_write_to_cluster(
            vector_to_write.data(),
            vector_to_write.size() * 4,
            address,
            {},
            rows_to_exclude_for_dram_broadcast,
            cols_to_exclude_for_dram_broadcast,
            "LARGE_WRITE_TLB");
        device.wait_for_non_mmio_flush();

        for (const auto i : target_devices) {
            for (const auto& core : device.get_soc_descriptor(i).workers) {
                if (rows_to_exclude.find(core.y) != rows_to_exclude.end()) {
                    continue;
                }
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(i, core), address, vector_to_write.size() * 4, "LARGE_READ_TLB");
                ASSERT_EQ(vector_to_write, readback_vec) << "Vector read back from core " << core.x << "-" << core.y
                                                         << "does not match what was broadcasted";
                device.write_to_device(
                    zeros.data(),
                    zeros.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, core),
                    address,
                    "LARGE_WRITE_TLB");  // Clear any written data
                readback_vec = {};
            }
            for (int chan = 0; chan < device.get_soc_descriptor(i).get_num_dram_channels(); chan++) {
                const auto& core = device.get_soc_descriptor(i).get_core_for_dram_channel(chan, 0);
                test_utils::read_data_from_device(
                    device, readback_vec, tt_cxy_pair(i, core), address, vector_to_write.size() * 4, "LARGE_READ_TLB");
                ASSERT_EQ(vector_to_write, readback_vec)
                    << "Vector read back from DRAM core " << i << " " << core.x << "-" << core.y
                    << " does not match what was broadcasted " << size;
                device.write_to_device(
                    zeros.data(),
                    zeros.size() * sizeof(std::uint32_t),
                    tt_cxy_pair(i, core),
                    address,
                    "LARGE_WRITE_TLB");  // Clear any written data
                readback_vec = {};
            }
        }
        // Wait for data to be cleared before writing next block
        device.wait_for_non_mmio_flush();
    }
    device.close_device();
}

/**
 * Copied from the Wormhole test.
 */
TEST(SiliconDriverBH, SysmemTestWithPcie) {
    auto target_devices = get_target_devices();

    Cluster cluster(
        1,      // one "host memory channel",
        false,  // skip driver allocs - no (don't skip)
        true,   // clean system resources - yes
        true);  // perform harvesting - yes

    set_params_for_remote_txn(cluster);
    cluster.start_device(tt_device_params{});  // no special parameters

    const chip_id_t mmio_chip_id = 0;
    const auto PCIE = cluster.get_soc_descriptor(mmio_chip_id).pcie_cores.at(0);
    const tt_cxy_pair PCIE_CORE(mmio_chip_id, PCIE.x, PCIE.y);
    const size_t test_size_bytes = 0x4000;  // Arbitrarilly chosen, but small size so the test runs quickly.

    uint8_t* sysmem = (uint8_t*)cluster.host_dma_address(0, 0, 0);
    ASSERT_NE(sysmem, nullptr);

    uint64_t base_address = cluster.get_pcie_base_addr_from_device(mmio_chip_id);

    // Buffer that we will use to read sysmem into, then write sysmem from.
    std::vector<uint8_t> buffer(test_size_bytes, 0x0);

    // Step 1: Fill sysmem with random bytes.
    test_utils::fill_with_random_bytes(sysmem, test_size_bytes);

    // Step 2: Read sysmem into buffer.
    cluster.read_from_device(&buffer[0], PCIE_CORE, base_address, buffer.size(), "REG_TLB");

    // Step 3: Verify that buffer matches sysmem.
    ASSERT_EQ(buffer, std::vector<uint8_t>(sysmem, sysmem + test_size_bytes));

    // Step 4: Fill buffer with random bytes.
    test_utils::fill_with_random_bytes(&buffer[0], test_size_bytes);

    // Step 5: Write buffer into sysmem, overwriting what was there.
    cluster.write_to_device(&buffer[0], buffer.size(), PCIE_CORE, base_address, "REG_TLB");

    // Step 5b: Read back sysmem into a throwaway buffer.  The intent is to
    // ensure the write has completed before we check sysmem against buffer.
    std::vector<uint8_t> throwaway(test_size_bytes, 0x0);
    cluster.read_from_device(&throwaway[0], PCIE_CORE, base_address, throwaway.size(), "REG_TLB");

    // Step 6: Verify that sysmem matches buffer.
    ASSERT_EQ(buffer, std::vector<uint8_t>(sysmem, sysmem + test_size_bytes));
}

static bool is_iommu_available() {
    const size_t num_channels = 1;
    auto target_devices = get_target_devices();
    Cluster cluster(
        test_utils::GetAbsPath("tests/soc_descs/blackhole_140_arch_no_eth.yaml"),
        target_devices,
        num_channels,
        false,  // skip driver allocs - no (don't skip)
        true,   // clean system resources - yes
        true);  // perform harvesting - yes
    return cluster.get_tt_device(0)->get_pci_device()->is_iommu_enabled();
}

/**
 * Same idea as above, but with multiple channels of sysmem and random addresses.
 * The hardware mechanism is too slow to sweep the entire range.
 */
TEST(SiliconDriverBH, RandomSysmemTestWithPcie) {
    // How many hugepages will Blackhole CI systems allocate?  Hopefully zero,
    // and they'll have IOMMU instead.  But if not, let's assume 2.
    const size_t num_channels = is_iommu_available() ? 4 : 2;
    auto target_devices = get_target_devices();

    Cluster cluster(
        test_utils::GetAbsPath("tests/soc_descs/blackhole_140_arch_no_eth.yaml"),
        target_devices,
        num_channels,
        false,  // skip driver allocs - no (don't skip)
        true,   // clean system resources - yes
        true);  // perform harvesting - yes

    set_params_for_remote_txn(cluster);
    cluster.start_device(tt_device_params{});  // no special parameters

    const chip_id_t mmio_chip_id = 0;
    const auto PCIE = cluster.get_soc_descriptor(mmio_chip_id).pcie_cores.at(0);
    const tt_cxy_pair PCIE_CORE(mmio_chip_id, PCIE.x, PCIE.y);
    const size_t ONE_GIG = 1 << 30;
    const size_t num_tests = 0x20000;  // runs in a reasonable amount of time

    const uint64_t ALIGNMENT = sizeof(uint32_t);
    auto generate_aligned_address = [&](uint64_t lo, uint64_t hi) -> uint64_t {
        static std::random_device rd;
        static std::mt19937_64 gen(rd());
        std::uniform_int_distribution<uint64_t> dis(lo / ALIGNMENT, hi / ALIGNMENT);
        return dis(gen) * ALIGNMENT;
    };

    uint64_t base_address = cluster.get_pcie_base_addr_from_device(mmio_chip_id);
    for (size_t channel = 0; channel < num_channels; ++channel) {
        uint8_t* sysmem = (uint8_t*)cluster.host_dma_address(0, 0, channel);
        ASSERT_NE(sysmem, nullptr);

        test_utils::fill_with_random_bytes(sysmem, ONE_GIG);

        uint64_t lo = (ONE_GIG * channel);
        uint64_t hi = (lo + ONE_GIG) - 1;

        for (size_t i = 0; i < num_tests; ++i) {
            uint64_t address = generate_aligned_address(lo, hi);
            uint64_t noc_addr = base_address + address;
            uint64_t sysmem_address = address - lo;

            ASSERT_GE(address, lo) << "Address too low";
            ASSERT_LE(address, hi) << "Address too high";
            ASSERT_EQ(address % ALIGNMENT, 0) << "Address not properly aligned";

            uint32_t value = 0;
            cluster.read_from_device(&value, PCIE_CORE, noc_addr, sizeof(uint32_t), "LARGE_READ_TLB");

            uint32_t expected = *reinterpret_cast<uint32_t*>(&sysmem[sysmem_address]);
            ASSERT_EQ(value, expected) << fmt::format("Mismatch at address {:#x}", address);
        }
    }
}

#endif