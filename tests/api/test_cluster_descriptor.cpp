
#include <gtest/gtest.h>

#include <string>
#include <vector>
#include <unordered_map>

#include "tests/test_utils/generate_cluster_desc.hpp"

#include "device/pcie/pci_device.hpp"
#include "device/tt_cluster_descriptor.h"


std::unique_ptr<tt_ClusterDescriptor> get_cluster_desc() {

    std::vector<int> pci_device_ids = PCIDevice::enumerate_devices();
    std::set<int> pci_device_ids_set (pci_device_ids.begin(), pci_device_ids.end());

    // TODO: This test requires knowledge of the device architecture, which should not be true.
    tt::ARCH device_arch = tt::ARCH::GRAYSKULL;
    if (!pci_device_ids.empty()) {
        int physical_device_id = pci_device_ids[0];
        PCIDevice pci_device (physical_device_id, 0);
        device_arch = pci_device.get_arch();
    }

    // TODO: Make this test work on a host system without any tt devices.
    if (pci_device_ids.empty()) {
        std::cout << "No Tenstorrent devices found. Skipping test." << std::endl;
        return nullptr;
    }

    // TODO: remove getting manually cluster descriptor from yaml.
    std::string yaml_path = test_utils::GetClusterDescYAML();
    // TODO: Remove the need to do this, allow default constructor to construct with all chips.
    std::unique_ptr<tt_ClusterDescriptor> cluster_desc;
    if (device_arch == tt::ARCH::GRAYSKULL) {
        cluster_desc = tt_ClusterDescriptor::create_for_grayskull_cluster(pci_device_ids_set, pci_device_ids);
    } else {
        cluster_desc = tt_ClusterDescriptor::create_from_yaml(yaml_path);
    }

    return cluster_desc;
}

TEST(ApiClusterDescriptorTest, BasicFunctionality) {

    std::unique_ptr<tt_ClusterDescriptor> cluster_desc = get_cluster_desc();

    if (cluster_desc == nullptr) {
        return;
    }

    std::unordered_set<chip_id_t> all_chips = cluster_desc->get_all_chips();
    std::unordered_map<chip_id_t, std::uint32_t> harvesting_for_chips = cluster_desc->get_harvesting_info();
    std::unordered_map<chip_id_t, eth_coord_t> eth_chip_coords = cluster_desc->get_chip_locations();
    std::unordered_map<chip_id_t, chip_id_t> local_chips_to_pci_device_id = cluster_desc->get_chips_with_mmio();
    std::unordered_set<chip_id_t> local_chips;
    for (auto [chip, _]: local_chips_to_pci_device_id) {
        local_chips.insert(chip);
    }
    std::unordered_set<chip_id_t> remote_chips;
    for (auto chip : all_chips) {
        if (local_chips.find(chip) == local_chips.end()) {
            remote_chips.insert(chip);
        }
    }
}

// A standard disjoint set data structure to track connected components.
class DisjointSet {
    public:
        void add_item(int item) {
            parent[item] = item;
        }

        int get_parent(int item) {
            while (parent[item] != item) {
                item = parent[item];
            }
            return item;
        }

        void merge(int item1, int item2) {
            int parent1 = get_parent(item1);
            int parent2 = get_parent(item2);
            parent[parent1] = parent2;
        }

        bool are_same_set(int item1, int item2) {
            return get_parent(item1) == get_parent(item2);
        }

        int get_num_sets() {
            std::unordered_set<int> sets;
            for (auto [item, _]: parent) {
                sets.insert(get_parent(item));
            }
            return sets.size();
        }

    private:
        std::unordered_map<int, int> parent;
};

// This tests fails on a machine with multiple cards.
// It works as long as all the devices that are discoverable are connected through ethernet.
// Our ClusterDescriptor doesn't have a notion of multiple unconnected clusters of cards.
TEST(ApiClusterDescriptorTest, SeparateClusters) {
    std::unique_ptr<tt_ClusterDescriptor> cluster_desc = get_cluster_desc();

    if (cluster_desc == nullptr) {
        return;
    }

    auto all_chips = cluster_desc->get_all_chips();
    DisjointSet chip_clusters;
    for (auto chip : all_chips) {
        chip_clusters.add_item(chip);
    }

    // Merge into clusters of chips.
    for (auto connection: cluster_desc->get_ethernet_connections()) {
        chip_id_t chip = connection.first;
        for (auto [channel, remote_chip_and_channel]: connection.second) {
            chip_id_t remote_chip = std::get<0>(remote_chip_and_channel);
            chip_clusters.merge(chip, remote_chip);
        }
    }

    // Print out the number of resulting clusters.
    std::cout << "Detected " << chip_clusters.get_num_sets() << " separate clusters." << std::endl;

    // Check that get_closes_mmio_capable_chip works.
    // Currently, it is expected that the following fails if there is more than 1 cluster.
    for (auto chip : all_chips) {
        chip_id_t closest_mmio_chip = cluster_desc->get_closest_mmio_capable_chip(chip);
        EXPECT_TRUE(chip_clusters.are_same_set(chip, closest_mmio_chip));
    }
}