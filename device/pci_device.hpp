/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>
#include <vector>

#include "device/tt_arch_types.h"
#include "architecture_implementation.h"

static uint32_t GS_BAR0_WC_MAPPING_SIZE = (156<<20) + (10<<21) + (18<<24);
static uint32_t BH_BAR0_WC_MAPPING_SIZE = 188<<21; // Defines the address for WC region. addresses 0 to BH_BAR0_WC_MAPPING_SIZE are in WC, above that are UC

static const uint32_t BH_NOC_NODE_ID_OFFSET = 0x1FD04044;
static const uint32_t GS_WH_ARC_SCRATCH_6_OFFSET = 0x1FF30078;

// See /vendor_ip/synopsys/052021/bh_pcie_ctl_gen5/export/configuration/DWC_pcie_ctl.h
static const uint64_t UNROLL_ATU_OFFSET_BAR = 0x1200;

// BAR0 size for Blackhole, used to determine whether write block should use BAR0 or BAR4
const uint64_t BAR0_BH_SIZE = 512 * 1024 * 1024;

typedef std::uint32_t DWORD;

class TTDevice {
public:
    TTDevice(int device_id, int logical_device_id);
    ~TTDevice();
    TTDevice(const TTDevice&) = delete; // copy
    void operator = (const TTDevice&) = delete; // copy assignment
    
    void write_block(uint64_t byte_addr, uint64_t num_bytes, const uint8_t* buffer_addr);
    void read_block(uint64_t byte_addr, uint64_t num_bytes, uint8_t* buffer_addr);
    void write_regs(uint32_t byte_addr, uint32_t word_len, const void *data);
    void write_regs(volatile uint32_t *dest, const uint32_t *src, uint32_t word_len);
    void read_regs(uint32_t byte_addr, uint32_t word_len, void *data);
    void write_tlb_reg(uint32_t byte_addr, std::uint64_t value_lower, std::uint64_t value_upper, std::uint32_t tlb_cfg_reg_size);

    void open_hugepage_per_host_mem_ch(uint32_t num_host_mem_channels);
    bool reset_board();
    tt::umd::architecture_implementation* get_architecture_implementation() const { return architecture_implementation.get(); }

    int device_id;
    int logical_id;
    int device_fd = -1;

    // PCIe device info
    std::uint32_t numa_node;
    std::uint16_t pcie_device_id;
    int pcie_revision_id;

    // BAR and regs mapping setup
    std::vector<int> device_fd_per_host_ch;
    void *bar0_uc = nullptr;
    std::size_t bar0_uc_size = 0;
    std::size_t bar0_uc_offset = 0;

    void *bar0_wc = nullptr;
    std::size_t bar0_wc_size = 0;

    void *bar2_uc = nullptr;
    std::size_t bar2_uc_size;

    void *bar4_wc = nullptr;
    std::uint64_t bar4_wc_size;

    void *system_reg_mapping = nullptr;
    std::size_t system_reg_mapping_size;

    // These two are currently not used.
    void *system_reg_wc_mapping = nullptr;
    std::size_t system_reg_wc_mapping_size;

    std::uint32_t system_reg_start_offset;  // Registers >= this are system regs, use the mapping.
    std::uint32_t system_reg_offset_adjust; // This is the offset of the first reg in the system reg mapping.

    // int sysfs_config_fd = -1;    // not used
    std::uint32_t read_checking_offset;

    tt::ARCH get_arch() const;
    
private:
    void get_pcie_info();
    void setup_device();
    void close_device();
    void drop();

    bool reset_by_sysfs();
    bool reset_by_ioctl();

    template <typename T>
    T* get_register_address(std::uint32_t register_offset);

    tt::ARCH arch;
    std::unique_ptr<tt::umd::architecture_implementation> architecture_implementation;
};
