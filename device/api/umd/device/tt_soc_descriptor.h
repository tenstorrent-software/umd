/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <cstddef>
#include <cstdint>
#include <iostream>
#include <map>
#include <string>
#include <unordered_map>
#include <vector>

#include "fmt/core.h"
#include "tt_xy_pair.h"
#include "umd/device/coordinate_manager.h"
#include "umd/device/tt_core_coordinates.h"
#include "umd/device/tt_xy_pair.h"
#include "umd/device/types/arch.h"

namespace YAML {
class Node;
}

std::string format_node(tt_xy_pair xy);

tt_xy_pair format_node(std::string str);

//! SocNodeDescriptor contains information regarding the Node/Core
/*!
    Should only contain relevant configuration for SOC
*/
struct CoreDescriptor {
    tt_xy_pair coord = tt_xy_pair(0, 0);
    CoreType type;

    std::size_t l1_size = 0;
};

//! tt_SocDescriptor contains information regarding the SOC configuration targetted.
/*!
    Should only contain relevant configuration for SOC
*/
class tt_SocDescriptor {
public:
    tt::ARCH arch;
    tt_xy_pair grid_size;
    tt_xy_pair physical_grid_size;
    tt_xy_pair worker_grid_size;
    std::unordered_map<tt_xy_pair, CoreDescriptor> cores;
    std::vector<tt_xy_pair> arc_cores;
    std::vector<tt_xy_pair> workers;
    std::vector<tt_xy_pair> harvested_workers;
    std::vector<tt_xy_pair> pcie_cores;
    std::unordered_map<int, int> worker_log_to_routing_x;
    std::unordered_map<int, int> worker_log_to_routing_y;
    std::unordered_map<int, int> routing_x_to_worker_x;
    std::unordered_map<int, int> routing_y_to_worker_y;
    std::vector<std::vector<tt_xy_pair>> dram_cores;                             // per channel list of dram cores
    std::unordered_map<tt_xy_pair, std::tuple<int, int>> dram_core_channel_map;  // map dram core to chan/subchan
    std::vector<tt_xy_pair> ethernet_cores;                                      // ethernet cores (index == channel id)
    std::unordered_map<tt_xy_pair, int> ethernet_core_channel_map;
    std::vector<std::size_t> trisc_sizes;  // Most of software stack assumes same trisc size for whole chip..
    std::string device_descriptor_file_path = std::string("");

    bool has(tt_xy_pair input) { return cores.find(input) != cores.end(); }

    int overlay_version;
    int unpacker_version;
    int dst_size_alignment;
    int packer_version;
    int worker_l1_size;
    int eth_l1_size;
    bool noc_translation_id_enabled;
    uint64_t dram_bank_size;

    int get_num_dram_channels() const;
    bool is_worker_core(const tt_xy_pair &core) const;
    tt_xy_pair get_core_for_dram_channel(int dram_chan, int subchannel) const;
    bool is_ethernet_core(const tt_xy_pair &core) const;

    // Default constructor. Creates uninitialized object with public access to all of its attributes.
    tt_SocDescriptor() = default;
    // Constructor used to build object from device descriptor file.
    tt_SocDescriptor(
        std::string device_descriptor_path,
        const std::size_t tensix_harvesting_mask = 0,
        const std::size_t dram_harvesting_mask = 0);

    // Copy constructor
    tt_SocDescriptor(const tt_SocDescriptor &other) :
        arch(other.arch),
        grid_size(other.grid_size),
        physical_grid_size(other.physical_grid_size),
        worker_grid_size(other.worker_grid_size),
        cores(other.cores),
        arc_cores(other.arc_cores),
        workers(other.workers),
        harvested_workers(other.harvested_workers),
        pcie_cores(other.pcie_cores),
        worker_log_to_routing_x(other.worker_log_to_routing_x),
        worker_log_to_routing_y(other.worker_log_to_routing_y),
        routing_x_to_worker_x(other.routing_x_to_worker_x),
        routing_y_to_worker_y(other.routing_y_to_worker_y),
        dram_cores(other.dram_cores),
        dram_core_channel_map(other.dram_core_channel_map),
        ethernet_cores(other.ethernet_cores),
        ethernet_core_channel_map(other.ethernet_core_channel_map),
        trisc_sizes(other.trisc_sizes),
        device_descriptor_file_path(other.device_descriptor_file_path),
        overlay_version(other.overlay_version),
        unpacker_version(other.unpacker_version),
        dst_size_alignment(other.dst_size_alignment),
        packer_version(other.packer_version),
        worker_l1_size(other.worker_l1_size),
        eth_l1_size(other.eth_l1_size),
        noc_translation_id_enabled(other.noc_translation_id_enabled),
        dram_bank_size(other.dram_bank_size),
        coordinate_manager(other.coordinate_manager) {}

    // CoreCoord conversions.
    tt::umd::CoreCoord to(const tt::umd::CoreCoord core_coord, const CoordSystem coord_system);

    static std::string get_soc_descriptor_path(tt::ARCH arch);

private:
    void create_coordinate_manager(const std::size_t tensix_harvesting_mask, const std::size_t dram_harvesting_mask);
    void load_core_descriptors_from_device_descriptor(YAML::Node &device_descriptor_yaml);
    void load_soc_features_from_device_descriptor(YAML::Node &device_descriptor_yaml);

    // TODO: change this to unique pointer as soon as copying of tt_SocDescriptor
    // is not needed anymore. Soc descriptor and coordinate manager should be
    // created once per chip.
    std::shared_ptr<CoordinateManager> coordinate_manager = nullptr;
};

// Allocates a new soc descriptor on the heap. Returns an owning pointer.
// std::unique_ptr<tt_SocDescriptor> load_soc_descriptor_from_yaml(std::string device_descriptor_file_path);
