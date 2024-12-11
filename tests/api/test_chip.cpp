// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

// This file holds Chip specific API examples.

#include <gtest/gtest.h>

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include "fmt/xchar.h"
#include "tests/test_utils/generate_cluster_desc.hpp"

// TODO: change to tt_cluster
#include "umd/device/architecture_implementation.h"
#include "umd/device/cluster.h"
#include "umd/device/tt_cluster_descriptor.h"

using namespace tt::umd;

inline tt_cxy_pair get_tensix_chip_core_coord(const std::unique_ptr<Cluster>& umd_cluster) {
    chip_id_t any_mmio_chip = *umd_cluster->get_target_mmio_device_ids().begin();
    const tt_SocDescriptor& soc_desc = umd_cluster->get_soc_descriptor(any_mmio_chip);
    tt_xy_pair core = soc_desc.workers[0];
    return tt_cxy_pair(any_mmio_chip, core);
}

inline std::unique_ptr<Cluster> get_cluster() {
    std::vector<int> pci_device_ids = PCIDevice::enumerate_devices();
    // TODO: Make this test work on a host system without any tt devices.
    if (pci_device_ids.empty()) {
        return nullptr;
    }
    return std::unique_ptr<Cluster>(new Cluster());
}

// TODO: Once default auto TLB setup is in, check it is setup properly.
TEST(ApiChipTest, ManualTLBConfiguration) {
    std::unique_ptr<Cluster> umd_cluster = get_cluster();

    if (umd_cluster == nullptr || umd_cluster->get_target_device_ids().empty()) {
        GTEST_SKIP() << "No chips present on the system. Skipping test.";
    }

    // Expect to throw for remote chip for any worker core
    auto remote_chips = umd_cluster->get_target_remote_device_ids();
    if (!remote_chips.empty()) {
        chip_id_t any_remote_chip = *remote_chips.begin();
        const tt_SocDescriptor& soc_desc = umd_cluster->get_soc_descriptor(any_remote_chip);
        tt_xy_pair core = soc_desc.workers[0];
        EXPECT_THROW(umd_cluster->get_static_tlb_writer(tt_cxy_pair(any_remote_chip, core)), std::runtime_error);
    }

    // Expect to throw for non configured mmio chip.
    chip_id_t any_mmio_chip = *umd_cluster->get_target_mmio_device_ids().begin();
    const tt_SocDescriptor& soc_desc = umd_cluster->get_soc_descriptor(any_mmio_chip);
    tt_xy_pair core = soc_desc.workers[0];
    EXPECT_THROW(umd_cluster->get_static_tlb_writer(tt_cxy_pair(any_mmio_chip, core)), std::runtime_error);

    // TODO: This should be part of TTDevice interface, not Cluster or Chip.
    // Configure TLBs.
    std::function<int(tt_xy_pair)> get_static_tlb_index = [&](tt_xy_pair core) -> int {
        // TODO: Make this per arch.
        bool is_worker_core = soc_desc.is_worker_core(core);
        if (!is_worker_core) {
            return -1;
        }
        return core.x +
               core.y * umd_cluster->get_tt_device(any_mmio_chip)->get_architecture_implementation()->get_grid_size_x();
    };

    std::int32_t c_zero_address = 0;

    // Each MMIO chip has it's own set of TLBs, so needs its own configuration.
    for (chip_id_t mmio_chip : umd_cluster->get_target_mmio_device_ids()) {
        const tt_SocDescriptor& soc_desc = umd_cluster->get_soc_descriptor(mmio_chip);
        for (tt_xy_pair core : soc_desc.workers) {
            umd_cluster->configure_tlb(mmio_chip, core, get_static_tlb_index(core), c_zero_address);
        }

        umd_cluster->setup_core_to_tlb_map(mmio_chip, get_static_tlb_index);
    }

    // Expect not to throw for now configured mmio chip, same one as before.
    EXPECT_NO_THROW(umd_cluster->get_static_tlb_writer(tt_cxy_pair(any_mmio_chip, core)));

    // Expect to throw for non worker cores.
    tt_xy_pair dram_core = soc_desc.dram_cores[0][0];
    EXPECT_THROW(umd_cluster->get_static_tlb_writer(tt_cxy_pair(any_mmio_chip, dram_core)), std::runtime_error);
    if (!soc_desc.ethernet_cores.empty()) {
        tt_xy_pair eth_core = soc_desc.ethernet_cores[0];
        EXPECT_THROW(umd_cluster->get_static_tlb_writer(tt_cxy_pair(any_mmio_chip, eth_core)), std::runtime_error);
    }
}

// TODO: Move to test_chip
TEST(ApiChipTest, SimpleAPIShowcase) {
    std::unique_ptr<Cluster> umd_cluster = get_cluster();

    if (umd_cluster == nullptr || umd_cluster->get_target_device_ids().empty()) {
        GTEST_SKIP() << "No chips present on the system. Skipping test.";
    }

    chip_id_t chip_id = umd_cluster->get_cluster_description()->get_chips_with_mmio().begin()->first;

    // TODO: In future, will be accessed through tt::umd::Chip api.
    umd_cluster->get_pcie_base_addr_from_device(chip_id);
    umd_cluster->get_num_host_channels(chip_id);
}

// TODO: Re-enable once we debug why it doesn't work #362
// // This tests puts a specific core into reset and then deasserts it using default deassert value
// // It reads back the risc reset reg to validate
// TEST(ApiChipTest, DeassertRiscResetOnCore) {
//     std::unique_ptr<Cluster> umd_cluster = get_cluster();

//     if (umd_cluster == nullptr || umd_cluster->get_target_device_ids().empty()) {
//         GTEST_SKIP() << "No chips present on the system. Skipping test.";
//     }

//     tt_cxy_pair chip_core_coord = get_tensix_chip_core_coord(umd_cluster);

//     umd_cluster->assert_risc_reset_at_core(chip_core_coord);
//     umd_cluster->l1_membar(chip_core_coord.chip, "LARGE_WRITE_TLB");
//     umd_cluster->deassert_risc_reset_at_core(chip_core_coord);
//     umd_cluster->l1_membar(chip_core_coord.chip, "LARGE_WRITE_TLB");

//     uint32_t soft_reset_reg_addr = 0xFFB121B0;
//     uint32_t expected_risc_reset_val = static_cast<uint32_t>(TENSIX_DEASSERT_SOFT_RESET);
//     uint32_t risc_reset_val;
//     umd_cluster->read_from_device(&risc_reset_val, chip_core_coord, soft_reset_reg_addr, sizeof(uint32_t),
//     "REG_TLB"); EXPECT_EQ(expected_risc_reset_val, risc_reset_val);
// }

// // This tests puts a specific core into reset and then specifies a legal deassert value
// // It reads back the risc reset reg to validate
// TEST(ApiChipTest, SpecifyLegalDeassertRiscResetOnCore) {
//     std::unique_ptr<Cluster> umd_cluster = get_cluster();

//     if (umd_cluster == nullptr || umd_cluster->get_target_device_ids().empty()) {
//         GTEST_SKIP() << "No chips present on the system. Skipping test.";
//     }

//     tt_cxy_pair chip_core_coord = get_tensix_chip_core_coord(umd_cluster);

//     umd_cluster->assert_risc_reset_at_core(chip_core_coord);
//     TensixSoftResetOptions deassert_val = ALL_TRISC_SOFT_RESET | TensixSoftResetOptions::STAGGERED_START;
//     umd_cluster->deassert_risc_reset_at_core(chip_core_coord, deassert_val);
//     umd_cluster->l1_membar(chip_core_coord.chip, "LARGE_WRITE_TLB");

//     uint32_t soft_reset_reg_addr = 0xFFB121B0;
//     uint32_t risc_reset_val;
//     umd_cluster->read_from_device(&risc_reset_val, chip_core_coord, soft_reset_reg_addr, sizeof(uint32_t),
//     "REG_TLB"); EXPECT_EQ(static_cast<uint32_t>(deassert_val), risc_reset_val);
// }

// // // This tests puts a specific core into reset and then specifies an illegal deassert value
// // // It reads back the risc reset reg to validate that reset reg is in a legal state
// TEST(ApiChipTest, SpecifyIllegalDeassertRiscResetOnCore) {
//     std::unique_ptr<Cluster> umd_cluster = get_cluster();

//     if (umd_cluster == nullptr || umd_cluster->get_target_device_ids().empty()) {
//         GTEST_SKIP() << "No chips present on the system. Skipping test.";
//     }

//     tt_cxy_pair chip_core_coord = get_tensix_chip_core_coord(umd_cluster);

//     umd_cluster->assert_risc_reset_at_core(chip_core_coord);

//     TensixSoftResetOptions deassert_val = static_cast<TensixSoftResetOptions>(0xDEADBEEF);
//     umd_cluster->deassert_risc_reset_at_core(chip_core_coord, deassert_val);
//     umd_cluster->l1_membar(chip_core_coord.chip, "LARGE_WRITE_TLB");

//     uint32_t soft_reset_reg_addr = 0xFFB121B0;
//     uint32_t risc_reset_val;
//     umd_cluster->read_from_device(&risc_reset_val, chip_core_coord, soft_reset_reg_addr, sizeof(uint32_t),
//     "REG_TLB"); uint32_t expected_deassert_val = static_cast<uint32_t>(deassert_val & ALL_TENSIX_SOFT_RESET);
//     EXPECT_EQ(risc_reset_val, expected_deassert_val);
// }
