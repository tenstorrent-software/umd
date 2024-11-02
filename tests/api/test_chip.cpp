// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

// This file holds Chip specific API examples.

#include <gtest/gtest.h>
#include "fmt/xchar.h"

#include <algorithm>
#include <filesystem>
#include <string>
#include <vector>

#include "tests/test_utils/generate_cluster_desc.hpp"

// TODO: change to tt_cluster
#include "device/tt_device.h"
#include "device/tt_cluster_descriptor.h"

// TODO: write this test to work with Chip not whole Cluster.
using Cluster = tt_SiliconDevice;

inline std::unique_ptr<tt_ClusterDescriptor> get_cluster_desc() {
    // TODO: This should not be needed. And could be part of the cluster descriptor probably.
    // Note that cluster descriptor holds logical ids of chips.
    // Which are different than physical PCI ids, which are /dev/tenstorrent/N ones.
    // You have to see if physical PCIe is GS before constructing a cluster descriptor.
    std::vector<int> pci_device_ids = PCIDevice::enumerate_devices();
    std::set<int> pci_device_ids_set (pci_device_ids.begin(), pci_device_ids.end());

    tt::ARCH device_arch = tt::ARCH::GRAYSKULL;
    if (!pci_device_ids.empty()) {
        // TODO: This should be removed from the API, the driver itself should do it.
        int physical_device_id = pci_device_ids[0];
        // TODO: remove logical_device_id
        PCIDevice pci_device (physical_device_id, 0);
        device_arch = pci_device.get_arch();
    }

    // TODO: Make this test work on a host system without any tt devices.
    if (pci_device_ids.empty()) {
        std::cout << "No Tenstorrent devices found. Skipping test." << std::endl;
        return nullptr;
    }

    // TODO: Remove different branch for different archs
    std::unique_ptr<tt_ClusterDescriptor> cluster_desc;
    if (device_arch == tt::ARCH::GRAYSKULL) {
        cluster_desc = tt_ClusterDescriptor::create_for_grayskull_cluster(pci_device_ids_set, pci_device_ids);
    } else if (device_arch == tt::ARCH::BLACKHOLE) {
        std::string yaml_path = test_utils::GetAbsPath("blackhole_1chip_cluster.yaml");
        cluster_desc = tt_ClusterDescriptor::create_from_yaml(yaml_path);
    } else {
        // TODO: remove getting manually cluster descriptor from yaml.
        std::string yaml_path = test_utils::GetClusterDescYAML();
        cluster_desc = tt_ClusterDescriptor::create_from_yaml(yaml_path);
    }

    return cluster_desc;
}

inline std::unique_ptr<Cluster> get_cluster() {

    // TODO: This should not be needed. And could be part of the cluster descriptor probably.
    // Note that cluster descriptor holds logical ids of chips.
    // Which are different than physical PCI ids, which are /dev/tenstorrent/N ones.
    // You have to see if physical PCIe is GS before constructing a cluster descriptor.
    std::vector<int> pci_device_ids = PCIDevice::enumerate_devices();
    std::set<int> pci_device_ids_set (pci_device_ids.begin(), pci_device_ids.end());

    tt::ARCH device_arch = tt::ARCH::GRAYSKULL;
    if (!pci_device_ids.empty()) {
        // TODO: This should be removed from the API, the driver itself should do it.
        int physical_device_id = pci_device_ids[0];
        // TODO: remove logical_device_id
        PCIDevice pci_device (physical_device_id, 0);
        device_arch = pci_device.get_arch();
    }

    // TODO: Make this test work on a host system without any tt devices.
    if (pci_device_ids.empty()) {
        std::cout << "No Tenstorrent devices found. Skipping test." << std::endl;
        return nullptr;
    }

    std::string yaml_path;
    if (device_arch == tt::ARCH::GRAYSKULL) {
        yaml_path = "";
    } else if (device_arch == tt::ARCH::BLACKHOLE) {
        yaml_path = test_utils::GetAbsPath("blackhole_1chip_cluster.yaml");
    } else {
        // TODO: remove getting manually cluster descriptor from yaml.
        yaml_path = test_utils::GetClusterDescYAML();
    }
    // TODO: Remove the need to do this, allow default constructor to construct with all chips.
    std::unique_ptr<tt_ClusterDescriptor> cluster_desc = get_cluster_desc();
    std::unordered_set<int> detected_num_chips = cluster_desc->get_all_chips();

    // TODO: make this unordered vs set conversion not needed.
    std::set<chip_id_t> detected_num_chips_set (detected_num_chips.begin(), detected_num_chips.end());

    
    // TODO: This would be incorporated inside SocDescriptor.
    std::string soc_path;
    if (device_arch == tt::ARCH::GRAYSKULL) {
        soc_path = test_utils::GetAbsPath("tests/soc_descs/grayskull_10x12.yaml");
    } else if (device_arch == tt::ARCH::WORMHOLE_B0) {
        soc_path = test_utils::GetAbsPath("tests/soc_descs/wormhole_b0_8x10.yaml");
    } else if (device_arch == tt::ARCH::BLACKHOLE) {
        soc_path = test_utils::GetAbsPath("tests/soc_descs/blackhole_140_arch_no_eth.yaml");
    } else {
        throw std::runtime_error("Unsupported architecture");
    }


    // TODO: Don't pass each of these arguments.
    return std::unique_ptr<Cluster>(new Cluster(soc_path, device_arch == tt::ARCH::GRAYSKULL ? "" : yaml_path, detected_num_chips_set));
}

// TODO: Once default auto TLB setup is in, check it is setup properly.
TEST(ApiChipTest, ManualTLBConfiguration) {
    std::unique_ptr<Cluster> umd_cluster = get_cluster();

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
        return core.x + core.y * umd_cluster->get_pci_device(any_mmio_chip)->get_architecture_implementation()->get_grid_size_x();
    };

    std::int32_t c_zero_address = 0;

    // Each MMIO chip has it's own set of TLBs, so needs its own configuration.
    for (chip_id_t mmio_chip: umd_cluster->get_target_mmio_device_ids()) {

        const tt_SocDescriptor& soc_desc = umd_cluster->get_soc_descriptor(mmio_chip);
        for (tt_xy_pair core: soc_desc.workers) {
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
    chip_id_t chip_id = *umd_cluster->get_target_mmio_device_ids().begin();

    // TODO: In future, will be accessed through tt::umd::Chip api.
    umd_cluster->get_pcie_base_addr_from_device(chip_id);
    umd_cluster->get_num_host_channels(chip_id);
}
