/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <array>

#include "device/architecture_implementation.h"
#include "device/tlb.h"
#include <stdexcept>

namespace tt::umd {

namespace blackhole {

static constexpr auto TLB_2M_OFFSET = tlb_offsets{
    .local_offset = 0,
    .x_end = 43,
    .y_end = 49,
    .x_start = 55,
    .y_start = 61,
    .noc_sel = 67,
    .mcast = 69,
    .ordering = 70,
    .linked = 72,
    .static_vc = 73,
    // missing .stream_header
    .static_vc_end = 75};

static constexpr auto TLB_4G_OFFSET = tlb_offsets{
    .local_offset = 0,
    .x_end = 32,
    .y_end = 38,
    .x_start = 44,
    .y_start = 50,
    .noc_sel = 56,
    .mcast = 58,
    .ordering = 59,
    .linked = 61,
    .static_vc = 62,
    // missing .stream_header
    .static_vc_end = 64};

enum class arc_message_type {
    NOP = 0x11,  // Do nothing
    GET_AICLK = 0x34,
    ARC_GO_BUSY = 0x52,
    ARC_GO_SHORT_IDLE = 0x53,
    ARC_GO_LONG_IDLE = 0x54,
    ARC_GET_HARVESTING = 0x57,
    SET_ETH_DRAM_TRAINED_STATUS = 0x58,
    TEST = 0x90,
    SETUP_IATU_FOR_PEER_TO_PEER = 0x97,
    DEASSERT_RISCV_RESET = 0xba
};

// DEVICE_DATA
static constexpr std::array<xy_pair, 24> DRAM_LOCATIONS = {
    {{0, 0},
     {0, 1},
     {0, 11},
     {0, 2},
     {0, 10},
     {0, 3},
     {0, 9},
     {0, 4},
     {0, 8},
     {0, 5},
     {0, 7},
     {0, 6},
     {9, 0},
     {9, 1},
     {9, 11},
     {9, 2},
     {9, 10},
     {9, 3},
     {9, 9},
     {9, 4},
     {9, 8},
     {9, 5},
     {9, 7},
     {9, 6}}};

static constexpr std::array<xy_pair, 1> ARC_LOCATIONS = {{{8, 0}}};
static constexpr std::array<xy_pair, 1> PCI_LOCATIONS = {{{11, 0}}};
static constexpr std::array<xy_pair, 14> ETH_LOCATIONS = {
    // MT: First implementation without ETH
    // {{1, 1},
    //  {2, 1},
    //  {3, 1},
    //  {4, 1},
    //  {5, 1},
    //  {6, 1},
    //  {7, 1},
    //  {10, 1},
    //  {11, 1},
    //  {12, 1},
    //  {13, 1},
    //  {14, 1},
    //  {15, 1},
    //  {16, 1}}
     };
// Return to std::array instead of std::vector once we get std::span support in C++20
static const std::vector<uint32_t> T6_X_LOCATIONS = {1, 2, 3, 4, 5, 6, 7, 10, 11, 12, 13, 14, 15, 16};
static const std::vector<uint32_t> T6_Y_LOCATIONS = {2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const std::vector<uint32_t> HARVESTING_NOC_LOCATIONS = {};

static constexpr uint32_t STATIC_TLB_SIZE = 1024 * 1024;  // TODO: Copied from wormhole. Need to verify.

static constexpr xy_pair BROADCAST_LOCATION = {0, 0};  // TODO: Copied from wormhole. Need to verify.
static constexpr uint32_t BROADCAST_TLB_INDEX = 0;     // TODO: Copied from wormhole. Need to verify.
static constexpr uint32_t STATIC_TLB_CFG_ADDR = 0x1fc00000;

static constexpr uint32_t TLB_COUNT_2M = 202;
static constexpr uint32_t TLB_BASE_2M = 0; // 0 in BAR0
static constexpr uint32_t TLB_BASE_INDEX_2M = 0;
static constexpr uint32_t TLB_2M_SIZE = 2 * 1024 * 1024;

static constexpr uint32_t TLB_CFG_REG_SIZE_BYTES = 12;

static constexpr uint32_t TLB_COUNT_4G = 8;
static constexpr uint32_t TLB_BASE_4G = 0; // 0 in BAR4
static constexpr uint32_t TLB_BASE_INDEX_4G = TLB_COUNT_2M;
static constexpr uint64_t TLB_4G_SIZE = 4ULL * 1024ULL * 1024ULL * 1024ULL;
static constexpr uint64_t DYNAMIC_TLB_4G_SIZE = TLB_4G_SIZE;
static constexpr uint32_t DYNAMIC_TLB_4G_CFG_ADDR = STATIC_TLB_CFG_ADDR + (TLB_BASE_INDEX_4G * TLB_CFG_REG_SIZE_BYTES);
static constexpr uint32_t DYNAMIC_TLB_4G_BASE = TLB_BASE_4G;

static constexpr uint32_t DYNAMIC_TLB_COUNT = 16;

static constexpr uint32_t DYNAMIC_TLB_2M_SIZE = 2 * 1024 * 1024;
static constexpr uint32_t DYNAMIC_TLB_2M_CFG_ADDR = STATIC_TLB_CFG_ADDR + (TLB_BASE_INDEX_2M * TLB_CFG_REG_SIZE_BYTES);
static constexpr uint32_t DYNAMIC_TLB_2M_BASE = TLB_BASE_2M;

// REG_TLB for dynamic writes to registers. They are aligned with the kernel driver's WC/UC split.  But kernel driver
// uses different TLB's for these.
// Revisit for BH
static constexpr unsigned int REG_TLB = TLB_BASE_INDEX_2M + 191;

static constexpr uint32_t DYNAMIC_TLB_BASE_INDEX = TLB_BASE_INDEX_2M + 180;
static constexpr unsigned int MEM_LARGE_WRITE_TLB = TLB_BASE_INDEX_2M + 181;
static constexpr unsigned int MEM_LARGE_READ_TLB = TLB_BASE_INDEX_2M + 182;
static constexpr unsigned int MEM_SMALL_READ_WRITE_TLB = TLB_BASE_INDEX_2M + 183;

static constexpr uint32_t DRAM_CHANNEL_0_X = 0;
static constexpr uint32_t DRAM_CHANNEL_0_Y = 1;
static constexpr uint32_t DRAM_CHANNEL_0_PEER2PEER_REGION_START = 0x30000000;  // This is the last 256MB of DRAM

static constexpr uint32_t GRID_SIZE_X = 17;
static constexpr uint32_t GRID_SIZE_Y = 12;

// AXI Resets accessed through TLB
static constexpr uint32_t TENSIX_SM_TLB_INDEX = 188;
static constexpr uint32_t AXI_RESET_OFFSET = TLB_BASE_2M + TENSIX_SM_TLB_INDEX * TLB_2M_SIZE;
static constexpr uint32_t ARC_RESET_SCRATCH_OFFSET = AXI_RESET_OFFSET + 0x0060;
static constexpr uint32_t ARC_RESET_ARC_MISC_CNTL_OFFSET = AXI_RESET_OFFSET + 0x0100;

// MT: This is no longer valid for Blackhole. Review messages to ARC
static constexpr uint32_t ARC_CSM_OFFSET = 0x1FE80000;
static constexpr uint32_t ARC_CSM_MAILBOX_OFFSET = ARC_CSM_OFFSET + 0x783C4;
static constexpr uint32_t ARC_CSM_MAILBOX_SIZE_OFFSET = ARC_CSM_OFFSET + 0x784C4;

static constexpr uint32_t TENSIX_SOFT_RESET_ADDR = 0xFFB121B0;

static constexpr uint32_t MSG_TYPE_SETUP_IATU_FOR_PEER_TO_PEER = 0x97;

}  // namespace blackhole

class blackhole_implementation : public architecture_implementation {
   public:
    architecture get_architecture() const override { return architecture::blackhole; }
    uint32_t get_arc_message_arc_get_harvesting() const override {
        return static_cast<uint32_t>(blackhole::arc_message_type::ARC_GET_HARVESTING);
    }
    uint32_t get_arc_message_arc_go_busy() const override {
        return static_cast<uint32_t>(blackhole::arc_message_type::ARC_GO_BUSY);
    }
    uint32_t get_arc_message_arc_go_long_idle() const override {
        return static_cast<uint32_t>(blackhole::arc_message_type::ARC_GO_LONG_IDLE);
    }
    uint32_t get_arc_message_arc_go_short_idle() const override {
        return static_cast<uint32_t>(blackhole::arc_message_type::ARC_GO_SHORT_IDLE);
    }
    uint32_t get_arc_message_deassert_riscv_reset() const override {
        return static_cast<uint32_t>(blackhole::arc_message_type::DEASSERT_RISCV_RESET);
    }
    uint32_t get_arc_message_get_aiclk() const override {
        return static_cast<uint32_t>(blackhole::arc_message_type::GET_AICLK);
    }
    uint32_t get_arc_message_setup_iatu_for_peer_to_peer() const override {
        return static_cast<uint32_t>(blackhole::arc_message_type::SETUP_IATU_FOR_PEER_TO_PEER);
    }
    uint32_t get_arc_message_test() const override { return static_cast<uint32_t>(blackhole::arc_message_type::TEST); }
    uint32_t get_arc_csm_mailbox_offset() const override { throw std::runtime_error("Not supported for Blackhole arch"); return 0; }
    uint32_t get_arc_reset_arc_misc_cntl_offset() const override { return blackhole::ARC_RESET_ARC_MISC_CNTL_OFFSET; }
    uint32_t get_arc_reset_scratch_offset() const override { return blackhole::ARC_RESET_SCRATCH_OFFSET; }
    uint32_t get_dram_channel_0_peer2peer_region_start() const override {
        return blackhole::DRAM_CHANNEL_0_PEER2PEER_REGION_START;
    }
    uint32_t get_dram_channel_0_x() const override { return blackhole::DRAM_CHANNEL_0_X; }
    uint32_t get_dram_channel_0_y() const override { return blackhole::DRAM_CHANNEL_0_Y; }
    uint32_t get_broadcast_tlb_index() const override { return blackhole::BROADCAST_TLB_INDEX; }
    uint32_t get_dynamic_tlb_2m_base() const override { return blackhole::DYNAMIC_TLB_2M_BASE; }
    uint32_t get_dynamic_tlb_2m_size() const override { return blackhole::DYNAMIC_TLB_2M_SIZE; }
    uint32_t get_dynamic_tlb_16m_base() const override { throw std::runtime_error("No 16MB TLBs for Blackhole arch"); return 0; }
    uint32_t get_dynamic_tlb_16m_size() const override { throw std::runtime_error("No 16MB TLBs for Blackhole arch"); return 0; }
    uint32_t get_dynamic_tlb_16m_cfg_addr() const override { throw std::runtime_error("No 16MB TLBs for Blackhole arch"); return 0; }
    uint32_t get_mem_large_read_tlb() const override { return blackhole::MEM_LARGE_READ_TLB; }
    uint32_t get_mem_large_write_tlb() const override { return blackhole::MEM_LARGE_WRITE_TLB; }
    uint32_t get_static_tlb_cfg_addr() const override { return blackhole::STATIC_TLB_CFG_ADDR; }
    uint32_t get_static_tlb_size() const override { return blackhole::STATIC_TLB_SIZE;  }
    uint32_t get_reg_tlb() const override { return blackhole::REG_TLB; }
    uint32_t get_tlb_base_index_16m() const override { throw std::runtime_error("No 16MB TLBs for Blackhole arch"); return 0;  }
    uint32_t get_tensix_soft_reset_addr() const override { return blackhole::TENSIX_SOFT_RESET_ADDR; }
    uint32_t get_grid_size_x() const override { return blackhole::GRID_SIZE_X; }
    uint32_t get_grid_size_y() const override { return blackhole::GRID_SIZE_Y; }
    uint32_t get_tlb_cfg_reg_size_bytes() const override { return blackhole::TLB_CFG_REG_SIZE_BYTES; }
    const std::vector<uint32_t>& get_harvesting_noc_locations() const override {
        return blackhole::HARVESTING_NOC_LOCATIONS;
    }
    const std::vector<uint32_t>& get_t6_x_locations() const override { return blackhole::T6_X_LOCATIONS; }
    const std::vector<uint32_t>& get_t6_y_locations() const override { return blackhole::T6_Y_LOCATIONS; }

    std::tuple<xy_pair, xy_pair> multicast_workaround(xy_pair start, xy_pair end) const override;
    tlb_configuration get_tlb_configuration(uint32_t tlb_index) const override;
    std::optional<std::tuple<std::uint64_t, std::uint64_t>> describe_tlb(std::int32_t tlb_index) const override;
    std::pair<std::uint64_t, std::uint64_t> get_tlb_data(std::uint32_t tlb_index, const tlb_data& data) const override;
};

}  // namespace tt::umd
