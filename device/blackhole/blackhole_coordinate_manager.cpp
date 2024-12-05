/*
 * SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "umd/device/blackhole_coordinate_manager.h"

using namespace tt::umd;

BlackholeCoordinateManager::BlackholeCoordinateManager(
    const tt_xy_pair& tensix_grid_size,
    const std::vector<tt_xy_pair>& tensix_cores,
    const size_t tensix_harvesting_mask,
    const tt_xy_pair& dram_grid_size,
    const std::vector<tt_xy_pair>& dram_cores,
    const size_t dram_harvesting_mask,
    const tt_xy_pair& eth_grid_size,
    const std::vector<tt_xy_pair>& eth_cores,
    const tt_xy_pair& arc_grid_size,
    const std::vector<tt_xy_pair>& arc_cores,
    const tt_xy_pair& pcie_grid_size,
    const std::vector<tt_xy_pair>& pcie_cores) :
    CoordinateManager(
        tensix_grid_size,
        tensix_cores,
        tensix_harvesting_mask,
        dram_grid_size,
        dram_cores,
        dram_harvesting_mask,
        eth_grid_size,
        eth_cores,
        arc_grid_size,
        arc_cores,
        pcie_grid_size,
        pcie_cores) {
    this->identity_map_physical_cores();
    this->translate_tensix_coords();
    this->translate_dram_coords();
    this->translate_eth_coords();
    this->translate_arc_coords();
    this->translate_pcie_coords();
}

void BlackholeCoordinateManager::translate_tensix_coords() {
    size_t num_harvested_x = CoordinateManager::get_num_harvested(tensix_harvesting_mask);
    size_t grid_size_x = tensix_grid_size.x;
    size_t grid_size_y = tensix_grid_size.y;

    size_t logical_x = 0;
    size_t x_index = grid_size_x - num_harvested_x;
    for (size_t x = 0; x < grid_size_x; x++) {
        if (tensix_harvesting_mask & (1 << x)) {
            size_t y_index = 0;
            for (size_t y = 0; y < grid_size_y; y++) {
                const tt_xy_pair& physical_core = tensix_cores[x + y * grid_size_x];
                const tt_xy_pair& virtual_core = tensix_cores[x_index + y_index * grid_size_x];

                CoreCoord virtual_coord =
                    CoreCoord(virtual_core.x, virtual_core.y, CoreType::TENSIX, CoordSystem::VIRTUAL);

                add_core_translation(virtual_coord, physical_core);

                y_index++;
            }
            x_index++;
        } else {
            for (size_t y = 0; y < grid_size_y; y++) {
                const tt_xy_pair& tensix_core = tensix_cores[x + y * grid_size_x];
                const tt_xy_pair& virtual_core = tensix_cores[logical_x + y * grid_size_x];

                CoreCoord logical_coord = CoreCoord(logical_x, y, CoreType::TENSIX, CoordSystem::LOGICAL);
                add_core_translation(logical_coord, tensix_core);

                CoreCoord virtual_coord =
                    CoreCoord(virtual_core.x, virtual_core.y, CoreType::TENSIX, CoordSystem::VIRTUAL);
                add_core_translation(virtual_coord, tensix_core);
            }
            logical_x++;
        }
    }

    fill_tensix_physical_translated_mapping();
}

void BlackholeCoordinateManager::fill_tensix_physical_translated_mapping() {
    for (const tt_xy_pair& physical_core : tensix_cores) {
        const CoreCoord virtual_coord = from_physical_map.at({physical_core, CoordSystem::VIRTUAL});
        const CoreCoord translated_coord =
            CoreCoord(virtual_coord.x, virtual_coord.y, CoreType::TENSIX, CoordSystem::TRANSLATED);

        add_core_translation(translated_coord, physical_core);
    }
}

void BlackholeCoordinateManager::translate_dram_coords() {
    size_t num_harvested_banks = CoordinateManager::get_num_harvested(dram_harvesting_mask);

    size_t logical_x = 0;
    for (size_t x = 0; x < dram_grid_size.x; x++) {
        if (!(dram_harvesting_mask & (1 << x))) {
            for (size_t y = 0; y < dram_grid_size.y; y++) {
                const tt_xy_pair& dram_core = dram_cores[x * dram_grid_size.y + y];

                CoreCoord logical_coord = CoreCoord(logical_x, y, CoreType::DRAM, CoordSystem::LOGICAL);

                add_core_translation(logical_coord, dram_core);
            }
            logical_x++;
        }
    }

    for (size_t x = 0; x < dram_grid_size.x - num_harvested_banks; x++) {
        for (size_t y = 0; y < dram_grid_size.y; y++) {
            const tt_xy_pair& dram_core = dram_cores[x * dram_grid_size.y + y];
            CoreCoord dram_logical = CoreCoord(x, y, CoreType::DRAM, CoordSystem::LOGICAL);
            CoreCoord dram_virtual = CoreCoord(dram_core.x, dram_core.y, CoreType::DRAM, CoordSystem::VIRTUAL);

            const tt_xy_pair physical_pair = to_physical_map[dram_logical];

            add_core_translation(dram_virtual, physical_pair);
        }
    }

    size_t harvested_index = (dram_grid_size.x - num_harvested_banks) * dram_grid_size.y;
    for (size_t x = 0; x < dram_grid_size.x; x++) {
        if (dram_harvesting_mask & (1 << x)) {
            for (size_t y = 0; y < dram_grid_size.y; y++) {
                const tt_xy_pair& dram_core = dram_cores[x * dram_grid_size.y + y];
                const tt_xy_pair& virtual_core = dram_cores[harvested_index++];

                CoreCoord virtual_coord =
                    CoreCoord(virtual_core.x, virtual_core.y, CoreType::DRAM, CoordSystem::VIRTUAL);

                add_core_translation(virtual_coord, dram_core);
            }
        }
    }

    fill_dram_physical_translated_mapping();
}

void BlackholeCoordinateManager::fill_eth_physical_translated_mapping() {
    for (size_t x = 0; x < eth_grid_size.x; x++) {
        for (size_t y = 0; y < eth_grid_size.y; y++) {
            const size_t translated_x = x + blackhole::eth_translated_coordinate_start_x;
            const size_t translated_y = y + blackhole::eth_translated_coordinate_start_y;

            CoreCoord logical_coord = CoreCoord(x, y, CoreType::ETH, CoordSystem::LOGICAL);
            const tt_xy_pair physical_pair = to_physical_map[logical_coord];

            CoreCoord translated_coord = CoreCoord(translated_x, translated_y, CoreType::ETH, CoordSystem::TRANSLATED);

            add_core_translation(translated_coord, physical_pair);
        }
    }
}

void BlackholeCoordinateManager::fill_pcie_physical_translated_mapping() {
    CoreCoord logical_coord = CoreCoord(0, 0, CoreType::PCIE, CoordSystem::LOGICAL);

    const tt_xy_pair physical_pair = to_physical_map[logical_coord];

    CoreCoord translated_coord = CoreCoord(
        blackhole::pcie_translated_coordinate_start_x,
        blackhole::pcie_translated_coordinate_start_y,
        CoreType::PCIE,
        CoordSystem::TRANSLATED);

    add_core_translation(translated_coord, physical_pair);
}

void BlackholeCoordinateManager::map_column_of_dram_banks(
    const size_t start_bank, const size_t end_bank, const size_t x_coord) {
    size_t translated_y = blackhole::dram_translated_coordinate_start_y;
    for (size_t bank = start_bank; bank < end_bank; bank++) {
        for (size_t port = 0; port < blackhole::NUM_NOC_PORTS_PER_DRAM_BANK; port++) {
            CoreCoord logical_coord = CoreCoord(bank, port, CoreType::DRAM, CoordSystem::LOGICAL);
            const tt_xy_pair physical_pair = to_physical_map[logical_coord];

            CoreCoord translated_coord = CoreCoord(x_coord, translated_y, CoreType::DRAM, CoordSystem::TRANSLATED);

            add_core_translation(translated_coord, physical_pair);

            translated_y++;
        }
    }
}

void BlackholeCoordinateManager::fill_dram_physical_translated_mapping() {
    const std::vector<size_t> harvested_banks = CoordinateManager::get_harvested_indices(dram_harvesting_mask);

    if (harvested_banks.empty()) {
        map_column_of_dram_banks(0, blackhole::NUM_DRAM_BANKS / 2, blackhole::dram_translated_coordinate_start_x);
        map_column_of_dram_banks(
            blackhole::NUM_DRAM_BANKS / 2,
            blackhole::NUM_DRAM_BANKS,
            blackhole::dram_translated_coordinate_start_x + 1);
        return;
    }

    const size_t harvested_bank = harvested_banks[0];

    if (harvested_bank < blackhole::NUM_DRAM_BANKS / 2) {
        const size_t mirror_east_bank = harvested_bank + blackhole::NUM_DRAM_BANKS / 2;
        map_column_of_dram_banks(
            0, blackhole::NUM_DRAM_BANKS / 2 - 1, blackhole::dram_translated_coordinate_start_x + 1);
        map_column_of_dram_banks(
            blackhole::NUM_DRAM_BANKS / 2 - 1,
            blackhole::NUM_DRAM_BANKS - 1,
            blackhole::dram_translated_coordinate_start_x);
    } else {
        const size_t mirror_west_bank = harvested_bank - blackhole::NUM_DRAM_BANKS / 2;
        map_column_of_dram_banks(0, blackhole::NUM_DRAM_BANKS / 2, blackhole::dram_translated_coordinate_start_x);
        map_column_of_dram_banks(
            blackhole::NUM_DRAM_BANKS / 2,
            blackhole::NUM_DRAM_BANKS - 1,
            blackhole::dram_translated_coordinate_start_x + 1);
    }

    const size_t virtual_index = (dram_grid_size.x - 1) * dram_grid_size.y;
    const size_t physical_index = harvested_bank * dram_grid_size.y;

    const size_t harvested_bank_translated_x = blackhole::dram_translated_coordinate_start_x + 1;
    const size_t harvested_bank_translated_y =
        blackhole::dram_translated_coordinate_start_y + (dram_grid_size.x / 2 - 1) * dram_grid_size.y;

    for (size_t noc_port = 0; noc_port < dram_grid_size.y; noc_port++) {
        const tt_xy_pair& physical_core = dram_cores[physical_index + noc_port];
        const tt_xy_pair& virtual_core = dram_cores[virtual_index + noc_port];

        CoreCoord virtual_coord = CoreCoord(virtual_core.x, virtual_core.y, CoreType::DRAM, CoordSystem::VIRTUAL);

        add_core_translation(virtual_coord, physical_core);

        CoreCoord translated_coord = CoreCoord(
            harvested_bank_translated_x,
            harvested_bank_translated_y + noc_port,
            CoreType::DRAM,
            CoordSystem::TRANSLATED);

        add_core_translation(translated_coord, physical_core);
    }
}
