// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0
#include <assert.h>
#include <dirent.h>
#include <errno.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/mman.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "logger.hpp"
#include "umd/device/architecture_implementation.h"
#include "umd/device/chip/local_chip.h"
#include "umd/device/chip/remote_chip.h"
#include "umd/device/cluster.h"
#include "umd/device/driver_atomics.h"
#include "umd/device/tt_cluster_descriptor.h"
#include "umd/device/tt_core_coordinates.h"
#include "umd/device/tt_silicon_driver_common.hpp"
#include "umd/device/tt_soc_descriptor.h"
#include "umd/device/tt_xy_pair.h"
#include "umd/device/types/arch.h"
#include "yaml-cpp/yaml.h"

#include "x280_uapi.hpp"
#include "logger_.hpp"

using namespace tt;
using namespace tt::umd;

static std::unique_ptr<NOC> NOC_DRIVER;
static std::unique_ptr<NocWindow> SYSMEM;

#define ERROR(...) fmt::println("ERROR: " __VA_ARGS__)

namespace tt::umd {

const tt_SocDescriptor& ClusterX280::get_soc_descriptor(chip_id_t chip_id) const {
    return chips_.at(chip_id)->get_soc_descriptor();
}

std::unordered_map<chip_id_t, tt_SocDescriptor> ClusterX280::get_virtual_soc_descriptors() {
    std::unordered_map<chip_id_t, tt_SocDescriptor> soc_descs;
    for (const auto& chip : chips_) {
        soc_descs[chip.first] = chip.second->get_soc_descriptor();
    }
    return soc_descs;
}

bool ClusterX280::using_harvested_soc_descriptors() { return perform_harvesting_on_sdesc && performed_harvesting; }

std::unordered_map<tt_xy_pair, tt_xy_pair> ClusterX280::get_harvested_coord_translation_map(
    chip_id_t logical_device_id) {
    return harvested_coord_translation.at(logical_device_id);
}

std::unordered_map<chip_id_t, uint32_t> ClusterX280::get_harvesting_masks_for_soc_descriptors() {
    if (using_harvested_soc_descriptors()) {
        return harvested_rows_per_target;
    }
    std::unordered_map<chip_id_t, uint32_t> default_harvesting_masks = {};
    for (const auto chip : target_devices_in_cluster) {
        default_harvesting_masks.insert({chip, 0});
    }
    return default_harvesting_masks;
}

void ClusterX280::construct_cluster(
    const uint32_t& num_host_mem_ch_per_mmio_device,
    const bool skip_driver_allocs,
    const bool clean_system_resources,
    bool perform_harvesting,
    std::unordered_map<chip_id_t, uint32_t> simulated_harvesting_masks) {
    // Prefill the soc_descriptor_per_chip
    for (const auto& [chip_id, chip] : chips_) {
        soc_descriptor_per_chip.emplace(chip_id, chip->get_soc_descriptor());
    }

    perform_harvesting_on_sdesc = perform_harvesting;

    auto architecture_implementation = tt::umd::architecture_implementation::create(arch_name);
    harvested_coord_translation[0] = create_harvested_coord_translation(arch_name, true);

    if (arch_name == tt::ARCH::BLACKHOLE) {
        // Default harvesting info for Blackhole, describing no harvesting
        for (auto chip_id = target_devices_in_cluster.begin(); chip_id != target_devices_in_cluster.end(); chip_id++) {
            harvested_rows_per_target[*chip_id] = 0;   // get_harvested_noc_rows_for_chip(*chip_id);
            num_rows_harvested.insert({*chip_id, 0});  // Only set for broadcast TLB to get RISCS out of reset. We want
                                                       // all rows to have a reset signal sent.
            if (harvested_rows_per_target[*chip_id]) {
                performed_harvesting = true;
            }
        }
    }

    if (simulated_harvesting_masks.size()) {
        performed_harvesting = true;
        for (auto device_id = target_devices_in_cluster.begin(); device_id != target_devices_in_cluster.end();
             device_id++) {
            log_assert(
                simulated_harvesting_masks.find(*device_id) != simulated_harvesting_masks.end(),
                "Could not find harvesting mask for device_id {}",
                *device_id);
            harvested_rows_per_target[*device_id] = simulated_harvesting_masks.at(*device_id);
        }
    }

    if (perform_harvesting) {
        perform_harvesting_on_soc_descriptors();
    }
    populate_cores();

    // Default initialize host_address_params based on detected arch
    host_address_params = architecture_implementation->get_host_address_params();

    // Default initialize eth_interface_params based on detected arch
    eth_interface_params = architecture_implementation->get_eth_interface_params();

    // Default initialize noc_params based on detected arch
    noc_params = architecture_implementation->get_noc_params();
}

std::unique_ptr<Chip> ClusterX280::construct_chip_from_cluster(
    chip_id_t chip_id, tt_ClusterDescriptor* cluster_desc, tt_SocDescriptor& soc_desc) {
    if (cluster_desc->is_chip_mmio_capable(chip_id)) {
        return std::make_unique<LocalChip>(soc_desc);
    } else {
        return std::make_unique<RemoteChip>(soc_desc);
    }
}

std::unique_ptr<Chip> ClusterX280::construct_chip_from_cluster(chip_id_t chip_id, tt_ClusterDescriptor* cluster_desc) {
    tt::ARCH arch = cluster_desc->get_arch(chip_id);
    std::string soc_desc_path = tt_SocDescriptor::get_soc_descriptor_path(arch);
    // Note that initially soc_descriptors are not harvested, but will be harvested later if perform_harvesting is
    // true.
    // TODO: This should be changed, harvesting should be done in tt_socdescriptor's constructor and not as part of
    // cluster class.
    // uint32_t harvesting_info = cluster_desc->get_harvesting_info().at(chip_id);
    tt_SocDescriptor soc_desc = tt_SocDescriptor(soc_desc_path /*, harvesting_info*/);
    return construct_chip_from_cluster(chip_id, cluster_desc, soc_desc);
}

ClusterX280::ClusterX280(
    const std::string& sdesc_path,
    const std::set<chip_id_t>& target_devices,
    const uint32_t& num_host_mem_ch_per_mmio_device,
    const bool skip_driver_allocs,
    const bool clean_system_resources,
    bool perform_harvesting,
    std::unordered_map<chip_id_t, uint32_t> simulated_harvesting_masks)
{
    noc_window_config sysmem_noc_config{};
    sysmem_noc_config.addr = 0x0;
    sysmem_noc_config.x_end = 9;
    sysmem_noc_config.y_end = 11;

    // Ugh.
    NOC_DRIVER = std::make_unique<NOC>();
    SYSMEM = NOC_DRIVER->allocate_window(FOUR_GIGS, sysmem_noc_config);

    cluster_desc = tt_ClusterDescriptor::create();
    target_devices_in_cluster.insert(0);

    std::vector<std::string> core_types = {
        "ARC",
        "DRAM",
        "ACTIVE_ETH",
        "IDLE_ETH",
        "PCIE",
        "TENSIX",
        "ROUTER_ONLY",
        "HARVESTED",
        "ETH",
        "WORKER",
    };

    for (auto& chip_id : target_devices) {
        log_assert(
            cluster_desc->get_all_chips().find(chip_id) != cluster_desc->get_all_chips().end(),
            "Target device {} not present in current cluster!",
            chip_id);

        // Note that initially soc_descriptors are not harvested, but will be harvested later if perform_harvesting is
        // true.
        // TODO: This should be changed, harvesting should be done in tt_socdescriptor's constructor and not as part of
        // cluster class.
        tt_SocDescriptor soc_desc = tt_SocDescriptor(sdesc_path);
        log_assert(
            cluster_desc->get_arch(chip_id) == soc_desc.arch,
            "Passed soc descriptor has {} arch, but for chip id {} has arch {}",
            arch_to_str(soc_desc.arch),
            chip_id,
            arch_to_str(cluster_desc->get_arch(chip_id)));
    }

    chips_[0] = construct_chip_from_cluster(0, cluster_desc.get());
    arch_name = chips_.begin()->second->get_soc_descriptor().arch;

    construct_cluster(
        num_host_mem_ch_per_mmio_device,
        skip_driver_allocs,
        clean_system_resources,
        perform_harvesting,
        simulated_harvesting_masks);
}

void ClusterX280::configure_active_ethernet_cores_for_mmio_device(
    chip_id_t mmio_chip, const std::unordered_set<tt_xy_pair>& active_eth_cores_per_chip) {}

void ClusterX280::populate_cores() {
    uint32_t count = 0;
    for (const auto& [chip_id, chip] : chips_) {
        auto& soc_desc = chip->get_soc_descriptor();
        workers_per_chip.insert(
            {chip_id, std::unordered_set<tt_xy_pair>(soc_desc.workers.begin(), soc_desc.workers.end())});
        if (count == 0) {
            eth_cores = std::unordered_set<tt_xy_pair>(soc_desc.ethernet_cores.begin(), soc_desc.ethernet_cores.end());
            for (uint32_t dram_idx = 0; dram_idx < soc_desc.get_num_dram_channels(); dram_idx++) {
                dram_cores.insert(soc_desc.get_core_for_dram_channel(dram_idx, 0));
            }
        }
        count++;
    }
}

std::vector<int> ClusterX280::extract_rows_to_remove(
    const tt::ARCH& arch, const int worker_grid_rows, const int harvested_rows) {
    // Check if harvesting config is legal for GS and WH
    log_assert(
        !((harvested_rows & 1) || (harvested_rows & 64) || (harvested_rows & 0xFFFFF000)),
        "For grayskull and wormhole, only rows 1-5 and 7-11 can be harvested");
    std::vector<int> row_coordinates_to_remove;
    int row_coordinate = 0;
    int tmp = harvested_rows;
    while (tmp) {
        if (tmp & 1) {
            row_coordinates_to_remove.push_back(row_coordinate);
        }

        tmp = tmp >> 1;
        row_coordinate++;
    }
    return row_coordinates_to_remove;
}

void ClusterX280::remove_worker_row_from_descriptor(
    tt_SocDescriptor& full_soc_descriptor, const std::vector<int>& row_coordinates_to_remove) {
    std::vector<tt_xy_pair> workers_to_keep;
    for (auto worker = (full_soc_descriptor.workers).begin(); worker != (full_soc_descriptor.workers).end(); worker++) {
        if (find(row_coordinates_to_remove.begin(), row_coordinates_to_remove.end(), (*worker).y) ==
            row_coordinates_to_remove.end()) {
            workers_to_keep.push_back(*worker);
        } else {
            (full_soc_descriptor.harvested_workers).push_back(*worker);
            full_soc_descriptor.cores.at(*worker).type = CoreType::HARVESTED;
        }
    }
    full_soc_descriptor.workers = workers_to_keep;
    (full_soc_descriptor.worker_grid_size).y -= row_coordinates_to_remove.size();
    full_soc_descriptor.routing_y_to_worker_y = {};
    full_soc_descriptor.worker_log_to_routing_y = {};

    std::set<int> modified_y_coords = {};

    for (const auto& core : full_soc_descriptor.workers) {
        modified_y_coords.insert(core.y);
    }
    int logical_y_coord = 0;
    for (const auto& y_coord : modified_y_coords) {
        full_soc_descriptor.routing_y_to_worker_y.insert({y_coord, logical_y_coord});
        full_soc_descriptor.worker_log_to_routing_y.insert({logical_y_coord, y_coord});
        logical_y_coord++;
    }
}

void ClusterX280::harvest_rows_in_soc_descriptor(tt::ARCH arch, tt_SocDescriptor& sdesc, uint32_t harvested_rows) {
    uint32_t max_row_to_remove =
        (*std::max_element((sdesc.workers).begin(), (sdesc.workers).end(), [](const auto& a, const auto& b) {
            return a.y < b.y;
        })).y;
    std::vector<int> row_coordinates_to_remove = extract_rows_to_remove(arch, max_row_to_remove, harvested_rows);
    remove_worker_row_from_descriptor(sdesc, row_coordinates_to_remove);
}

void ClusterX280::perform_harvesting_on_soc_descriptors() {
    for (const auto& chip : harvested_rows_per_target) {
        harvest_rows_in_soc_descriptor(arch_name, chips_.at(chip.first)->get_soc_descriptor(), chip.second);
    }
}

std::unordered_map<tt_xy_pair, tt_xy_pair> ClusterX280::create_harvested_coord_translation(
    const tt::ARCH arch, bool identity_map) {
    log_assert(
        identity_map ? true : (arch != tt::ARCH::GRAYSKULL), "NOC Translation can only be performed for WH devices");
    std::unordered_map<tt_xy_pair, tt_xy_pair> translation_table = {};

    tt_xy_pair grid_size;
    std::vector<uint32_t> T6_x = {};
    std::vector<uint32_t> T6_y = {};
    std::vector<tt_xy_pair> ethernet = {};
    // Store device specific data for GS and WH depending on arch
    if (arch == tt::ARCH::GRAYSKULL) {
        grid_size = tt_xy_pair(13, 12);
        T6_x = {12, 1, 11, 2, 10, 3, 9, 4, 8, 5, 7, 6};
        T6_y = {11, 1, 10, 2, 9, 3, 8, 4, 7, 5};
    } else if (arch == tt::ARCH::BLACKHOLE) {
        grid_size = tt_xy_pair(17, 12);
        T6_x = {16, 1, 15, 2, 14, 3, 13, 4, 12, 5, 11, 6, 10, 7};
        T6_y = {11, 2, 10, 3, 9, 4, 8, 5, 7, 6};
    } else {
        grid_size = tt_xy_pair(10, 12);
        T6_x = {1, 2, 3, 4, 6, 7, 8, 9};
        T6_y = {1, 2, 3, 4, 5, 7, 8, 9, 10, 11};
        // clang-format off
        ethernet = {{1, 0}, {2, 0}, {3, 0}, {4, 0}, {6, 0}, {7, 0}, {8, 0}, {9, 0},
                    {1, 6}, {2, 6}, {3, 6}, {4, 6}, {6, 6}, {7, 6}, {8, 6}, {9, 6}};
        // clang-format on
    }

    if (identity_map) {
        // When device is initialized, assume no harvesting and create an identity map for cores
        // This flow is always used for GS, since there is no hardware harvesting
        for (int x = 0; x < grid_size.x; x++) {
            for (int y = 0; y < grid_size.y; y++) {
                tt_xy_pair curr_core = tt_xy_pair(x, y);
                translation_table.insert({curr_core, curr_core});
            }
        }
        return translation_table;
    }

    // If this function is called with identity_map = false, we have perform NOC translation
    // This can only happen for WH devices
    // Setup coord translation for workers. Map all worker cores
    for (int x = 0; x < grid_size.x; x++) {
        for (int y = 0; y < grid_size.y; y++) {
            tt_xy_pair curr_core = tt_xy_pair(x, y);

            if (std::find(T6_x.begin(), T6_x.end(), x) != T6_x.end() &&
                std::find(T6_y.begin(), T6_y.end(), y) != T6_y.end()) {
                // This is a worker core. Apply translation for WH.
                tt_xy_pair harvested_worker;
                if (x >= 1 && x <= 4) {
                    harvested_worker.x = x + 17;
                } else if (x <= 9 && x > 5) {
                    harvested_worker.x = x + 16;
                } else {
                    log_assert(false, "Invalid WH worker x coord {} when creating translation tables.", x);
                }

                if (y >= 1 && y <= 5) {
                    harvested_worker.y = y + 17;
                } else if (y <= 11 && y > 6) {
                    harvested_worker.y = y + 16;
                } else {
                    log_assert(false, "Invalid WH worker y coord {} when creating translation tables.", y);
                }
                translation_table.insert({curr_core, harvested_worker});
            }

            else if (std::find(ethernet.begin(), ethernet.end(), curr_core) != ethernet.end()) {
                // This is an eth core. Apply translation for WH.
                tt_xy_pair harvested_eth_core;
                if (x >= 1 && x <= 4) {
                    harvested_eth_core.x = x + 17;
                } else if (x <= 9 && x > 5) {
                    harvested_eth_core.x = x + 16;
                } else {
                    log_assert(false, "Invalid WH eth_core x coord {} when creating translation tables.", x);
                }

                if (y == 0) {
                    harvested_eth_core.y = y + 16;
                } else if (y == 6) {
                    harvested_eth_core.y = y + 11;
                } else {
                    log_assert(false, "Invalid WH eth_core y coord {} when creating translation tables.", y);
                }
                translation_table.insert({curr_core, harvested_eth_core});
            }

            else {
                // All other cores for WH are not translated in case of harvesting.
                translation_table.insert({curr_core, curr_core});
            }
        }
    }
    return translation_table;
}

void ClusterX280::translate_to_noc_table_coords(chip_id_t device_id, std::size_t& r, std::size_t& c) {
    auto translated_coords = harvested_coord_translation[device_id].at(tt_xy_pair(c, r));
    c = translated_coords.x;
    r = translated_coords.y;
}

void ClusterX280::broadcast_pcie_tensix_risc_reset(chip_id_t chip_id, const TensixSoftResetOptions& soft_resets) {
    log_debug(LogSiliconDriver, "ClusterX280::broadcast_tensix_risc_reset");

    TTDevice* tt_device = get_tt_device(chip_id);

    auto valid = soft_resets & ALL_TENSIX_SOFT_RESET;

    log_debug(
        LogSiliconDriver,
        "== For all tensix set soft-reset for {} risc cores.",
        TensixSoftResetOptionsToString(valid).c_str());

    auto architecture_implementation = tt_device->get_architecture_implementation();

    // TODO: this is clumsy and difficult to read
    auto [soft_reset_reg, _] = tt_device->set_dynamic_tlb_broadcast(
        architecture_implementation->get_reg_tlb(),
        architecture_implementation->get_tensix_soft_reset_addr(),
        harvested_coord_translation.at(chip_id),
        tt_xy_pair(0, 0),
        tt_xy_pair(
            architecture_implementation->get_grid_size_x() - 1,
            architecture_implementation->get_grid_size_y() - 1 - num_rows_harvested.at(chip_id)),
        TLB_DATA::Posted);
    tt_device->write_regs(soft_reset_reg, 1, &valid);
    tt_driver_atomics::sfence();
}

std::set<chip_id_t> ClusterX280::get_target_mmio_device_ids() {
    std::set<chip_id_t> s;
    s.insert(0);
    return s;
    return all_target_mmio_devices;
}

void ClusterX280::assert_risc_reset() { broadcast_tensix_risc_reset_to_cluster(TENSIX_ASSERT_SOFT_RESET); }

void ClusterX280::deassert_risc_reset() { broadcast_tensix_risc_reset_to_cluster(TENSIX_DEASSERT_SOFT_RESET); }

void ClusterX280::deassert_risc_reset_at_core(tt_cxy_pair core, const TensixSoftResetOptions& soft_resets) {
    // Get Target Device to query soc descriptor and determine location in cluster
    uint32_t target_device = core.chip;
    log_assert(
        std::find(
            get_soc_descriptor(target_device).workers.begin(), get_soc_descriptor(target_device).workers.end(), core) !=
                get_soc_descriptor(target_device).workers.end() ||
            std::find(
                get_soc_descriptor(target_device).ethernet_cores.begin(),
                get_soc_descriptor(target_device).ethernet_cores.end(),
                core) != get_soc_descriptor(target_device).ethernet_cores.end(),
        "Cannot deassert reset on a non-tensix or harvested core");
    bool target_is_mmio_capable = cluster_desc->is_chip_mmio_capable(target_device);
    if (target_is_mmio_capable) {
        send_tensix_risc_reset_to_core(core, soft_resets);
    } else {
        log_assert(arch_name != tt::ARCH::BLACKHOLE, "Can't issue access to remote core in BH");
    }
}

void ClusterX280::assert_risc_reset_at_core(tt_cxy_pair core) {
    // Get Target Device to query soc descriptor and determine location in cluster
    uint32_t target_device = core.chip;
    log_assert(
        std::find(
            get_soc_descriptor(target_device).workers.begin(), get_soc_descriptor(target_device).workers.end(), core) !=
                get_soc_descriptor(target_device).workers.end() ||
            std::find(
                get_soc_descriptor(target_device).ethernet_cores.begin(),
                get_soc_descriptor(target_device).ethernet_cores.end(),
                core) != get_soc_descriptor(target_device).ethernet_cores.end(),
        "Cannot assert reset on a non-tensix or harvested core");
    bool target_is_mmio_capable = cluster_desc->is_chip_mmio_capable(target_device);
    if (target_is_mmio_capable) {
        send_tensix_risc_reset_to_core(core, TENSIX_ASSERT_SOFT_RESET);
    } else {
    }
}

std::unordered_set<chip_id_t> ClusterX280::get_all_chips_in_cluster() { return cluster_desc->get_all_chips(); }

int ClusterX280::get_number_of_chips_in_cluster() {
    // Returns the number of chips seen in the network descriptor
    return cluster_desc->get_all_chips().size();
}

tt_ClusterDescriptor* ClusterX280::get_cluster_description() { return cluster_desc.get(); }

// Can be used before instantiating a silicon device
int ClusterX280::detect_number_of_chips() {
    auto available_device_ids = detect_available_device_ids();
    return available_device_ids.size();
}

// Can be used before instantiating a silicon device
std::vector<chip_id_t> ClusterX280::detect_available_device_ids() { return {0}; }

std::optional<std::tuple<uint64_t, uint32_t>> ClusterX280::get_tlb_data_from_target(const tt_cxy_pair& target) {
    // This is a total mess - horrible hacks to support a horrible API.
    // Once I get it working, I'll clean it up to something that is actually sane.
    noc_window_config config{};
    config.x_end = target.x;
    config.y_end = target.y;
    auto window = NOC_DRIVER->allocate_window(TWO_MEGS, config);
    uint8_t* base = window->data();
    size_t size = window->size();
    noc_windows_.push_back(std::move(window));
    return std::make_tuple((uint64_t)base, size);
}


std::function<void(uint64_t, uint32_t, const uint8_t*)> ClusterX280::get_fast_pcie_static_tlb_write_callable(int device_id) {
    const auto callable = [](uint64_t byte_addr, uint32_t num_bytes, const uint8_t* src) {
        uint8_t* dst = reinterpret_cast<uint8_t*>(byte_addr);
        std::memcpy(dst, src, num_bytes);
    };

    return callable;
}

tt::Writer ClusterX280::get_static_tlb_writer(tt_cxy_pair target) {
    noc_window_config config{};
    config.x_end = target.x;
    config.y_end = target.y;
    auto window = NOC_DRIVER->allocate_window(TWO_MEGS, config);

    uint8_t* base = window->data();
    uint32_t tlb_size = window->size();

    // HACK: tt::Writer should really own it, but I don't want to do that right now.
    noc_windows_.push_back(std::move(window));
    return tt::Writer(base, tlb_size);
}


std::map<int, int> ClusterX280::get_clocks() {
    std::map<int, int> clock_freq_map;
    clock_freq_map[0] = 0;
    return clock_freq_map;
}

ClusterX280::~ClusterX280() {
    cluster_desc.reset();
    tlb_config_map.clear();
}

// JMS: exposing this as part of the API was design error
void ClusterX280::configure_tlb(chip_id_t logical_device_id, tt_xy_pair core, int32_t tlb_index, uint64_t address, uint64_t ordering)
{
    static bool once = true;
    if (once)
    {
        once = false;
        ERROR("Called configure_tlb, but this function doesn't do anything");
    }
}

void ClusterX280::set_fallback_tlb_ordering_mode(const std::string& fallback_tlb, uint64_t ordering) {}


void ClusterX280::bar_write32(int logical_device_id, uint32_t addr, uint32_t data)
{
    ERROR("X280 has no BARs");
    return;
}

uint32_t ClusterX280::bar_read32(int logical_device_id, uint32_t addr)
{
    ERROR("X280 has no BARs");
    return -1;
}

// Returns 0 if everything was OK
int ClusterX280::pcie_arc_msg(
    int logical_device_id,
    uint32_t msg_code,
    bool wait_for_done,
    uint32_t arg0,
    uint32_t arg1,
    int timeout,
    uint32_t* return_3,
    uint32_t* return_4) {
    return -1;
}

// Returns broken rows as bits set to 1 in 'memory' and 'logic'
uint32_t ClusterX280::get_harvested_noc_rows(uint32_t harvesting_mask) { return 0; }


uint32_t ClusterX280::get_harvested_noc_rows_for_chip(int logical_device_id) {
    return get_harvested_noc_rows(0);
}

void* ClusterX280::host_dma_address(uint64_t offset, chip_id_t, uint16_t channel) const {
    assert(SYSMEM);

    uint8_t* base = SYSMEM->data();
    offset += (1ULL << 30) * channel;

    return base + offset;
}

// Wrapper for throwing more helpful exception when not-enabled pci intf is accessed.
inline TTDevice* ClusterX280::get_tt_device(chip_id_t device_id) const {
    ERROR("get_tt_device is going away!");
    throw std::runtime_error("Don't do that");
    return nullptr;
}

void ClusterX280::pcie_broadcast_write(
    chip_id_t chip,
    const void* mem_ptr,
    uint32_t size_in_bytes,
    uint32_t addr,
    const tt_xy_pair& start,
    const tt_xy_pair& end,
    const std::string& fallback_tlb) {
    throw std::runtime_error("Don't do that");
}

inline bool tensix_or_eth_in_broadcast(
    const std::set<uint32_t>& cols_to_exclude,
    const tt::umd::architecture_implementation* architecture_implementation) {
    bool found_tensix_or_eth = false;
    for (const auto& col : architecture_implementation->get_t6_x_locations()) {
        found_tensix_or_eth |= (cols_to_exclude.find(col) == cols_to_exclude.end());
    }
    return found_tensix_or_eth;
}

inline bool valid_tensix_broadcast_grid(
    const std::set<uint32_t>& rows_to_exclude,
    const std::set<uint32_t>& cols_to_exclude,
    const tt::umd::architecture_implementation* architecture_implementation) {
    bool t6_bcast_rows_complete = true;
    bool t6_bcast_rows_empty = true;

    for (const auto& row : architecture_implementation->get_t6_y_locations()) {
        t6_bcast_rows_complete &= (rows_to_exclude.find(row) == rows_to_exclude.end());
        t6_bcast_rows_empty &= (rows_to_exclude.find(row) != rows_to_exclude.end());
    }
    return t6_bcast_rows_complete || t6_bcast_rows_empty;
}

// Kind of a puzzling name...
void ClusterX280::ethernet_broadcast_write(
    const void* mem_ptr,
    uint32_t size_in_bytes,
    uint64_t address,
    const std::set<chip_id_t>& chips_to_exclude,
    const std::set<uint32_t>& rows_to_exclude,
    std::set<uint32_t>& cols_to_exclude,
    const std::string& fallback_tlb,
    bool use_virtual_coords)
{
    for (const auto& [coord, core] : get_soc_descriptor(0).cores) {
        auto x = coord.x;
        auto y = coord.y;

        if (cols_to_exclude.count(x)) {
            continue;
        }
        if (rows_to_exclude.count(y)) {
            continue;
        }
        if (core.type == CoreType::HARVESTED) {
            continue;
        }
        auto value = *reinterpret_cast<const uint32_t*>(mem_ptr);
        UMD_INFO("Writing to device at value {:#x}", value);
        write_to_device(mem_ptr, size_in_bytes, tt_cxy_pair(0, x, y), address, fallback_tlb);
        read_from_device(&value, tt_cxy_pair(0, x, y), address, 4, fallback_tlb);
        UMD_INFO("Read back value {:#x}", value);
    }
}

// TODO: you really need to sort this out, in this new X280 context that you've
// thrust this code into, it makes even less sense than it did before.
void ClusterX280::broadcast_write_to_cluster(
    const void* mem_ptr,
    uint32_t size_in_bytes,
    uint64_t address,
    const std::set<chip_id_t>& chips_to_exclude,
    std::set<uint32_t>& rows_to_exclude,
    std::set<uint32_t>& cols_to_exclude,
    const std::string& fallback_tlb)
{
#if 0
    auto architecture_implementation = tt::umd::architecture_implementation::create(arch_name);
    if (cols_to_exclude.find(0) == cols_to_exclude.end() or cols_to_exclude.find(9) == cols_to_exclude.end()) {
        log_assert(
            !tensix_or_eth_in_broadcast(cols_to_exclude, architecture_implementation.get()),
            "Cannot broadcast to tensix/ethernet and DRAM simultaneously on Blackhole.");
        if (cols_to_exclude.find(0) == cols_to_exclude.end()) {
            // When broadcast includes column zero do not exclude anything
            std::set<uint32_t> unsafe_rows = {};
            std::set<uint32_t> cols_to_exclude_for_col_0_bcast = cols_to_exclude;
            std::set<uint32_t> rows_to_exclude_for_col_0_bcast = rows_to_exclude;
            cols_to_exclude_for_col_0_bcast.insert(9);
            rows_to_exclude_for_col_0_bcast.insert(unsafe_rows.begin(), unsafe_rows.end());
            ethernet_broadcast_write(
                mem_ptr,
                size_in_bytes,
                address,
                chips_to_exclude,
                rows_to_exclude_for_col_0_bcast,
                cols_to_exclude_for_col_0_bcast,
                fallback_tlb,
                false);
        }
        if (cols_to_exclude.find(9) == cols_to_exclude.end()) {
            std::set<uint32_t> cols_to_exclude_for_col_9_bcast = cols_to_exclude;
            cols_to_exclude_for_col_9_bcast.insert(0);
            ethernet_broadcast_write(
                mem_ptr,
                size_in_bytes,
                address,
                chips_to_exclude,
                rows_to_exclude,
                cols_to_exclude_for_col_9_bcast,
                fallback_tlb,
                false);
        }
    } else
#endif
    {
        ethernet_broadcast_write(
            mem_ptr,
            size_in_bytes,
            address,
            chips_to_exclude,
            rows_to_exclude,
            cols_to_exclude,
            fallback_tlb,
            false);
    }
}

int ClusterX280::remote_arc_msg(
    int chip,
    uint32_t msg_code,
    bool wait_for_done,
    uint32_t arg0,
    uint32_t arg1,
    int timeout,
    uint32_t* return_3,
    uint32_t* return_4) {
    return -1;
}

void ClusterX280::write_to_sysmem(const void* src, uint32_t size, uint64_t addr, uint16_t channel, chip_id_t)
{
    assert(SYSMEM);

    uint8_t* base = SYSMEM->data();
    size_t offset = (1ULL << 30) * channel;
    if (offset + addr + size >= SYSMEM->size()) {
        log_fatal(
            "Attempted to write to sysmem at offset {} with size {} which exceeds the size of sysmem {}",
            offset,
            size,
            SYSMEM->size());
    }
    memcpy(base + offset + addr, src, size);
}

void ClusterX280::read_from_sysmem(void* dst, uint64_t addr, uint16_t channel, uint32_t size, chip_id_t)
{
    assert(SYSMEM);

    uint8_t* base = SYSMEM->data();
    size_t offset = (1ULL << 30) * channel;

    if (offset + addr + size >= SYSMEM->size()) {
        log_fatal(
            "Attempted to read from sysmem at offset {} with size {} which exceeds the size of sysmem {}",
            offset,
            size,
            SYSMEM->size());
    }

    memcpy(dst, base + offset + addr, size);
}

void ClusterX280::set_membar_flag(
    const chip_id_t chip,
    const std::unordered_set<tt_xy_pair>& cores,
    const uint32_t barrier_value,
    const uint32_t barrier_addr,
    const std::string& fallback_tlb) {
    tt_driver_atomics::sfence();  // Ensure that writes before this do not get reordered
    std::unordered_set<tt_xy_pair> cores_synced = {};
    std::vector<uint32_t> barrier_val_vec = {barrier_value};
    for (const auto& core : cores) {
        write_to_device(
            barrier_val_vec.data(),
            barrier_val_vec.size() * sizeof(uint32_t),
            tt_cxy_pair(chip, core),
            barrier_addr,
            fallback_tlb);
    }
    tt_driver_atomics::sfence();  // Ensure that all writes in the Host WC buffer are flushed
    while (cores_synced.size() != cores.size()) {
        for (const auto& core : cores) {
            if (cores_synced.find(core) == cores_synced.end()) {
                uint32_t readback_val;
                read_from_device(&readback_val, tt_cxy_pair(chip, core), barrier_addr, sizeof(uint32_t), fallback_tlb);
                if (readback_val == barrier_value) {
                    cores_synced.insert(core);
                } else {
                    log_trace(
                        LogSiliconDriver,
                        "Waiting for core {} to recieve mem bar flag {} in function",
                        core.str(),
                        barrier_value);
                }
            }
        }
    }
    // Ensure that reads or writes after this do not get reordered.
    // Reordering can cause races where data gets transferred before the barrier has returned
    tt_driver_atomics::mfence();
}

void ClusterX280::insert_host_to_device_barrier(
    const chip_id_t chip,
    const std::unordered_set<tt_xy_pair>& cores,
    const uint32_t barrier_addr,
    const std::string& fallback_tlb) {
    set_membar_flag(chip, cores, tt_MemBarFlag::SET, barrier_addr, fallback_tlb);
    set_membar_flag(chip, cores, tt_MemBarFlag::RESET, barrier_addr, fallback_tlb);
}

void ClusterX280::init_membars() {
    for (const auto& chip : target_devices_in_cluster) {
        if (cluster_desc->is_chip_mmio_capable(chip)) {
            // TODO: you should make sure these are all populated properly
            // Why are these address params in different places?
            set_membar_flag(chip, workers_per_chip.at(chip), tt_MemBarFlag::RESET, l1_address_params.tensix_l1_barrier_base, "LARGE_WRITE_TLB");
            set_membar_flag(chip, eth_cores, tt_MemBarFlag::RESET, l1_address_params.eth_l1_barrier_base, "LARGE_WRITE_TLB");
            set_membar_flag(chip, dram_cores, tt_MemBarFlag::RESET, dram_address_params.DRAM_BARRIER_BASE, "LARGE_WRITE_TLB");
        }
    }
}

void ClusterX280::l1_membar(
    const chip_id_t chip, const std::string& fallback_tlb, const std::unordered_set<tt_xy_pair>& cores) {
    if (cluster_desc->is_chip_mmio_capable(chip)) {
        const auto& all_workers = workers_per_chip.at(chip);
        const auto& all_eth = eth_cores;
        if (cores.size()) {
            // Insert barrier on specific cores with L1
            std::unordered_set<tt_xy_pair> workers_to_sync = {};
            std::unordered_set<tt_xy_pair> eth_to_sync = {};

            for (const auto& core : cores) {
                if (all_workers.find(core) != all_workers.end()) {
                    workers_to_sync.insert(core);
                } else if (all_eth.find(core) != all_eth.end()) {
                    eth_to_sync.insert(core);
                } else {
                    log_fatal("Can only insert an L1 Memory barrier on Tensix or Ethernet cores.");
                }
            }
            insert_host_to_device_barrier( chip, workers_to_sync, l1_address_params.tensix_l1_barrier_base, fallback_tlb);
            insert_host_to_device_barrier(chip, eth_to_sync, l1_address_params.eth_l1_barrier_base, fallback_tlb);
        } else {
            // Insert barrier on all cores with L1
            insert_host_to_device_barrier(chip, all_workers, l1_address_params.tensix_l1_barrier_base, fallback_tlb);
            insert_host_to_device_barrier(chip, all_eth, l1_address_params.eth_l1_barrier_base, fallback_tlb);
        }
    }
}

void ClusterX280::dram_membar(
    const chip_id_t chip, const std::string& fallback_tlb, const std::unordered_set<tt_xy_pair>& cores) {
    if (cluster_desc->is_chip_mmio_capable(chip)) {
        if (cores.size()) {
            for (const auto& core : cores) {
                log_assert(
                    dram_cores.find(core) != dram_cores.end(), "Can only insert a DRAM Memory barrier on DRAM cores.");
            }
            insert_host_to_device_barrier(chip, cores, dram_address_params.DRAM_BARRIER_BASE, fallback_tlb);
        } else {
            // Insert Barrier on all DRAM Cores
            insert_host_to_device_barrier(chip, dram_cores, dram_address_params.DRAM_BARRIER_BASE, fallback_tlb);
        }
    }
}

void ClusterX280::dram_membar(
    const chip_id_t chip, const std::string& fallback_tlb, const std::unordered_set<uint32_t>& channels) {
    if (cluster_desc->is_chip_mmio_capable(chip)) {
        if (channels.size()) {
            std::unordered_set<tt_xy_pair> dram_cores_to_sync = {};
            for (const auto& chan : channels) {
                dram_cores_to_sync.insert(get_soc_descriptor(chip).get_core_for_dram_channel(chan, 0));
            }
            insert_host_to_device_barrier(
                chip, dram_cores_to_sync, dram_address_params.DRAM_BARRIER_BASE, fallback_tlb);
        } else {
            // Insert Barrier on all DRAM Cores
            insert_host_to_device_barrier(chip, dram_cores, dram_address_params.DRAM_BARRIER_BASE, fallback_tlb);
        }
    }
}

void ClusterX280::read_from_device(void* mem_ptr, tt_cxy_pair core, uint64_t addr, uint32_t size, const std::string& fallback_tlb)
{
    auto x = core.x;
    auto y = core.y;
    auto dst = reinterpret_cast<uint8_t*>(mem_ptr);
    NOC_DRIVER->read(x, y, addr, dst, size);
}

void ClusterX280::write_to_device(const void* mem_ptr, uint32_t size, tt_cxy_pair core, uint64_t addr, const std::string& fallback_tlb)
{
    auto x = core.x;
    auto y = core.y;
    auto src = reinterpret_cast<const uint8_t*>(mem_ptr);
    NOC_DRIVER->write_block(x, y, addr, src, size);
}

int ClusterX280::arc_msg(
    int logical_device_id,
    uint32_t msg_code,
    bool wait_for_done,
    uint32_t arg0,
    uint32_t arg1,
    int timeout,
    uint32_t* return_3,
    uint32_t* return_4) {
    log_assert(arch_name != tt::ARCH::BLACKHOLE, "ARC messages not supported in Blackhole");
    return 0;
}

void ClusterX280::send_tensix_risc_reset_to_core(const tt_cxy_pair& core, const TensixSoftResetOptions& soft_resets) {
    auto valid = soft_resets & ALL_TENSIX_SOFT_RESET;
    uint32_t valid_val = (std::underlying_type<TensixSoftResetOptions>::type)valid;
    write_to_device(&valid_val, sizeof(uint32_t), core, 0xFFB121B0, "REG_TLB");
    tt_driver_atomics::sfence();
}

// TODO: get rid of this in this form
void ClusterX280::broadcast_tensix_risc_reset_to_cluster(const TensixSoftResetOptions& soft_resets)
{
    auto valid = soft_resets & ALL_TENSIX_SOFT_RESET;
    uint32_t valid_val = (std::underlying_type<TensixSoftResetOptions>::type)valid;
    std::set<chip_id_t> chips_to_exclude = {};
    std::set<uint32_t> rows_to_exclude;
    std::set<uint32_t> columns_to_exclude;
    if (arch_name == tt::ARCH::BLACKHOLE) {
        rows_to_exclude = {0, 1};
        columns_to_exclude = {0, 8, 9};
    } else {
        rows_to_exclude = {0, 6};
        columns_to_exclude = {0, 5};
    }
    std::string fallback_tlb = "LARGE_WRITE_TLB";
    std::cout << "Broadcast reset to cluster" << std::endl;
    broadcast_write_to_cluster(
        &valid_val,
        sizeof(uint32_t),
        0xFFB121B0,
        chips_to_exclude,
        rows_to_exclude,
        columns_to_exclude,
        fallback_tlb);
}

std::set<chip_id_t> ClusterX280::get_target_remote_device_ids() { return target_remote_chips; }


void ClusterX280::start_device(const tt_device_params& device_params)
{
    init_membars();
}

void ClusterX280::close_device() {
    broadcast_tensix_risc_reset_to_cluster(TENSIX_ASSERT_SOFT_RESET);
}

void ClusterX280::set_device_l1_address_params(const tt_device_l1_address_params& l1_address_params_) {
    l1_address_params = l1_address_params_;
}

void ClusterX280::set_device_dram_address_params(const tt_device_dram_address_params& dram_address_params_) {
    dram_address_params = dram_address_params_;
}

void ClusterX280::set_driver_host_address_params(const tt_driver_host_address_params& host_address_params_) {
    host_address_params = host_address_params_;
}

void ClusterX280::set_driver_eth_interface_params(const tt_driver_eth_interface_params& eth_interface_params_) {
    eth_interface_params = eth_interface_params_;
}

void ClusterX280::setup_core_to_tlb_map(const chip_id_t logical_device_id, std::function<std::int32_t(tt_xy_pair)> mapping_function) {
}

uint32_t ClusterX280::get_num_dram_channels(uint32_t device_id) {
    return get_soc_descriptor(device_id).get_num_dram_channels();
}

uint64_t ClusterX280::get_dram_channel_size(uint32_t device_id, uint32_t channel) {
    return get_soc_descriptor(device_id).dram_bank_size;  // Space per channel is identical for now
}

uint32_t ClusterX280::get_num_host_channels(uint32_t device_id)
{
    // 1?  I stole a full DRAM bank, so we have 4.
    return 1;
}

uint32_t ClusterX280::get_host_channel_size(uint32_t device_id, uint32_t channel) {
    return 1 << 30;  // 1GB per channel
}

uint32_t ClusterX280::get_numa_node_for_pcie_device(uint32_t device_id) {
    return 0;
}

uint64_t ClusterX280::get_pcie_base_addr_from_device(const chip_id_t chip_id) const {
    return 0;
}

tt_version ClusterX280::get_ethernet_fw_version() const {
    return {0xffff, 0xff, 0xff};
}

}  // namespace tt::umd
