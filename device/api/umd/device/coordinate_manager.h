/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <map>
#include <set>
#include <vector>

#include "umd/device/tt_core_coordinates.h"
#include "umd/device/tt_xy_pair.h"
#include "umd/device/types/arch.h"

class CoordinateManager {
public:
    static std::shared_ptr<CoordinateManager> create_coordinate_manager(
        tt::ARCH arch,
        const tt_xy_pair& tensix_grid_size,
        const std::vector<tt_xy_pair>& tensix_cores,
        const size_t tensix_harvesting_mask,
        const tt_xy_pair& dram_grid_size,
        const std::vector<tt_xy_pair>& dram_cores,
        const size_t dram_harvesting_mask,
        const tt_xy_pair& eth_grid_size,
        const std::vector<tt_xy_pair>& eth_cores,
        const size_t eth_harvesting_mask,
        const tt_xy_pair& arc_grid_size,
        const std::vector<tt_xy_pair>& arc_cores,
        const tt_xy_pair& pcie_grid_size,
        const std::vector<tt_xy_pair>& pcie_cores);

    static std::shared_ptr<CoordinateManager> create_coordinate_manager(
        tt::ARCH arch,
        const size_t tensix_harvesting_mask = 0,
        const size_t dram_harvesting_mask = 0,
        const size_t eth_harvesting_mask = 0);

    static size_t get_num_harvested(const size_t harvesting_mask);

    static std::vector<size_t> get_harvested_indices(const size_t harvesting_mask);

    CoordinateManager(CoordinateManager& other) = default;

    tt::umd::CoreCoord translate_coord_to(const tt::umd::CoreCoord core_coord, const CoordSystem coord_system);

    std::vector<tt::umd::CoreCoord> get_cores(const CoreType core_type) const;
    tt_xy_pair get_grid_size(const CoreType core_type) const;

    std::vector<tt::umd::CoreCoord> get_harvested_cores(const CoreType core_type) const;
    tt_xy_pair get_harvested_grid_size(const CoreType core_type) const;

    virtual ~CoordinateManager() = default;

    size_t get_tensix_harvesting_mask() const;

    size_t get_dram_harvesting_mask() const;

    size_t get_eth_harvesting_mask() const;

private:
    static void assert_create_coordinate_manager(
        const tt::ARCH arch,
        const size_t tensix_harvesting_mask,
        const size_t dram_harvesting_mask,
        const size_t eth_harvesting_mask);

    const std::vector<tt_xy_pair>& get_physical_pairs(const CoreType core_type) const;
    std::vector<tt::umd::CoreCoord> get_all_physical_cores(const CoreType core_type) const;

protected:
    /*
     * Constructor for Coordinate Manager.
     * Tensix harvesting mask is supposed to be passed as original harvesting mask that is
     * returned from create-ethernet-map, so each bit is responsible for one row of the actual physical
     * row of the tensix cores on the chip. Harvesting mask is shuffled in constructor to match the NOC
     * layout of the tensix cores.
     */
    CoordinateManager(
        const tt_xy_pair& tensix_grid_size,
        const std::vector<tt_xy_pair>& tensix_cores,
        const size_t tensix_harvesting_mask,
        const tt_xy_pair& dram_grid_size,
        const std::vector<tt_xy_pair>& dram_cores,
        const size_t dram_harvesting_mask,
        const tt_xy_pair& eth_grid_size,
        const std::vector<tt_xy_pair>& eth_cores,
        const size_t eth_harvesting_mask,
        const tt_xy_pair& arc_grid_size,
        const std::vector<tt_xy_pair>& arc_cores,
        const tt_xy_pair& pcie_grid_size,
        const std::vector<tt_xy_pair>& pcie_cores);

    void initialize();

    virtual void shuffle_tensix_harvesting_mask(const std::vector<uint32_t>& harvesting_locations);

    virtual void translate_tensix_coords();
    virtual void translate_dram_coords();
    virtual void translate_eth_coords();
    virtual void translate_arc_coords();
    virtual void translate_pcie_coords();

    void identity_map_physical_cores();
    void add_core_translation(const tt::umd::CoreCoord& core_coord, const tt_xy_pair& physical_pair);

    virtual std::vector<tt::umd::CoreCoord> get_tensix_cores() const;
    virtual std::vector<tt::umd::CoreCoord> get_harvested_tensix_cores() const;
    virtual std::vector<tt::umd::CoreCoord> get_dram_cores() const;
    virtual std::vector<tt::umd::CoreCoord> get_harvested_dram_cores() const;
    virtual std::vector<tt::umd::CoreCoord> get_eth_cores() const;
    virtual std::vector<tt::umd::CoreCoord> get_harvested_eth_cores() const;
    virtual tt_xy_pair get_tensix_grid_size() const;
    virtual tt_xy_pair get_dram_grid_size() const;
    virtual tt_xy_pair get_eth_grid_size() const;
    virtual tt_xy_pair get_harvested_tensix_grid_size() const;
    virtual tt_xy_pair get_harvested_dram_grid_size() const;
    virtual tt_xy_pair get_harvested_eth_grid_size() const;

    /*
     * Fills the logical to translated mapping for the tensix cores.
     * By default, translated coordinates are the same as physical coordinates.
     * Derived coordinate managers that need to implement different mapping
     * should override this method. Wormhole and Blackhole coordinate managers
     * override this method to implement different mapping.
     */
    virtual void fill_tensix_physical_translated_mapping();

    /*
     * Fills the physical to translated mapping for the ethernet cores.
     * By default, translated coordinates are the same as physical coordinates.
     * Derived coordinate managers that need to implement different mapping
     * should override this method. Wormhole and Blackhole coordinate managers
     * override this method to implement different mapping.
     */
    virtual void fill_eth_physical_translated_mapping();

    /*
     * Fills the physical to translated mapping for the DRAM cores.
     * By default, translated coordinates are the same as physical coordinates.
     * Derived coordinate managers that need to implement different mapping
     * should override this method. Blackhole coordinate manager overrides
     * this method to implement different mapping.
     */
    virtual void fill_dram_physical_translated_mapping();

    /*
     * Fills the physical to translated mapping for the PCIE cores.
     * By default, translated coordinates are the same as physical coordinates.
     * Derived coordinate managers that need to implement different mapping
     * should override this method. Blackhole coordinate manager overrides
     * this method to implement different mapping.
     */
    virtual void fill_pcie_physical_translated_mapping();

    /*
     * Fills the physical to translated mapping for the ARC cores.
     * By default, translated coordinates are the same as physical coordinates.
     * Derived coordinate managers that need to implement different mapping
     * should override this method.
     */
    virtual void fill_arc_physical_translated_mapping();

    std::map<tt::umd::CoreCoord, tt_xy_pair> to_physical_map;
    std::map<std::pair<tt_xy_pair, CoordSystem>, tt::umd::CoreCoord> from_physical_map;

    tt_xy_pair tensix_grid_size;
    const std::vector<tt_xy_pair> tensix_cores;
    size_t tensix_harvesting_mask;
    const size_t physical_layout_tensix_harvesting_mask;

    tt_xy_pair dram_grid_size;
    const std::vector<tt_xy_pair> dram_cores;
    size_t dram_harvesting_mask;

    tt_xy_pair eth_grid_size;
    const std::vector<tt_xy_pair> eth_cores;
    const size_t eth_harvesting_mask;

    tt_xy_pair arc_grid_size;
    const std::vector<tt_xy_pair> arc_cores;

    tt_xy_pair pcie_grid_size;
    const std::vector<tt_xy_pair> pcie_cores;
};
