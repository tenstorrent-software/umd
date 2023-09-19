#pragma once

#include <cstdint>
#include <sys/mman.h>
#include <unistd.h>
#include <utility>
#include <vector>

#include "kmdif.h"
#include "ioctl.h"
#include "tt_soc_descriptor.h"
#include "tt_cluster_descriptor_types.h"

// Stash all the fields of TTDevice in TTDeviceBase to make moving simpler.
struct TTDeviceBase
{
    unsigned int index;

    int device_fd = -1;
    std::vector<int> device_fd_per_host_ch;
    void *bar0_uc = nullptr;
    std::size_t bar0_uc_size = 0;
    std::size_t bar0_uc_offset = 0;

    void *bar0_wc = nullptr;
    std::size_t bar0_wc_size = 0;

    void *system_reg_mapping = nullptr;
    std::size_t system_reg_mapping_size;

    void *system_reg_wc_mapping = nullptr;
    std::size_t system_reg_wc_mapping_size;

    std::uint32_t system_reg_start_offset;  // Registers >= this are system regs, use the mapping.
    std::uint32_t system_reg_offset_adjust; // This is the offset of the first reg in the system reg mapping.

    int sysfs_config_fd = -1;

    std::uint8_t pci_bus;
    std::uint8_t pci_device;
    std::uint8_t pci_function;

    unsigned int next_dma_buf = 0;

	DMAbuffer dma_completion_flag_buffer;  // When DMA completes, it writes to this buffer
	DMAbuffer dma_transfer_buffer;         // Buffer for large DMA transfers

    std::uint32_t max_dma_buf_size_log2;

    tenstorrent_get_device_info_out device_info;

    std::vector<DMAbuffer> dma_buffer_mappings;
};

struct TTDevice : TTDeviceBase
{
    static TTDevice open(unsigned int device_id);
    void open_hugepage_per_host_mem_ch(uint32_t num_host_mem_channels);
    ~TTDevice() { reset(); }

    TTDevice(const TTDevice&) = delete;
    void operator = (const TTDevice&) = delete;

    TTDevice(TTDevice &&that) : TTDeviceBase(std::move(that)) { that.drop(); }
    TTDevice &operator = (TTDevice &&that) {
        reset();

        *static_cast<TTDeviceBase*>(this) = std::move(that);
        that.drop();

        return *this;
    }

    void suspend_before_device_reset() {
        reset();
    }

    void resume_after_device_reset() {
        do_open();
    }

private:
    TTDevice() = default;

    void reset() {
        if (device_fd != -1) {
            close(device_fd);
        }

        if (bar0_wc != nullptr && bar0_wc != MAP_FAILED && bar0_wc != bar0_uc) {
            munmap(bar0_wc, bar0_wc_size);
        }

        if (bar0_uc != nullptr && bar0_uc != MAP_FAILED) {
            munmap(bar0_uc, bar0_uc_size);
        }

        if (system_reg_mapping != nullptr && system_reg_mapping != MAP_FAILED) {
            munmap(system_reg_mapping, system_reg_mapping_size);
        }

        for (auto &&buf : dma_buffer_mappings) {
            munmap(buf.pBuf, buf.size);
        }

        if (sysfs_config_fd != -1) {
            close(sysfs_config_fd);
        }

        drop();
    }

    void drop() {
        device_fd = -1;
        bar0_uc = nullptr;
        bar0_wc = nullptr;
        system_reg_mapping = nullptr;
        dma_buffer_mappings.clear();
        sysfs_config_fd = -1;
    }

    void do_open();
};

struct dynamic_tlb {
    uint32_t bar_offset;        // Offset that address is mapped to, within the PCI BAR.
    uint32_t remaining_size;    // Bytes remaining between bar_offset and end of the TLB.
};

bool is_wormhole(const uint16_t device_id);
bool is_grayskull(const uint16_t device_id);
bool is_wormhole(const tenstorrent_get_device_info_out &device_info);
bool is_wormhole_b0(const uint16_t device_id, const uint16_t revision_id);

int find_device(const uint16_t device_id);

DMAbuffer pci_allocate_dma_buffer(TTDevice *dev, uint32_t size);
void set_use_dma(bool msi, uint32_t dma_block_size_read_threshold_bytes, uint32_t dma_block_size_write_threshold_bytes);
void pcie_init_dma_transfer_turbo();
std::uint64_t pci_dma_buffer_get_physical_addr(DMAbuffer &dma_buffer);
std::uint64_t pci_dma_buffer_get_user_addr(DMAbuffer &dma_buffer);

dynamic_tlb set_dynamic_tlb(PCIdevice* dev, unsigned int tlb_index, tt_xy_pair start, tt_xy_pair end,
                            std::uint32_t address, bool multicast, std::unordered_map<chip_id_t, std::unordered_map<tt_xy_pair, tt_xy_pair>>& harvested_coord_translation, bool posted);
dynamic_tlb set_dynamic_tlb(PCIdevice *dev, unsigned int tlb_index, tt_xy_pair target, std::uint32_t address, std::unordered_map<chip_id_t, std::unordered_map<tt_xy_pair, tt_xy_pair>>& harvested_coord_translation, bool posted = false);
dynamic_tlb set_dynamic_tlb_broadcast(PCIdevice *dev, unsigned int tlb_index, std::uint32_t address, std::unordered_map<chip_id_t, std::unordered_map<tt_xy_pair, tt_xy_pair>>& harvested_coord_translation, uint32_t num_rows_harvested, bool posted = false);

void read_block(TTDevice *dev, uint32_t byte_addr, uint32_t num_bytes, uint64_t buffer_addr, uint32_t dma_buf_size);
void write_block(TTDevice *dev, uint32_t byte_addr, uint32_t num_bytes, uint64_t buffer_addr, uint32_t dma_buf_size);

void read_regs(TTDevice *dev, uint32_t byte_addr, uint32_t word_len, void *data);
void write_regs(TTDevice *dev, uint32_t byte_addr, uint32_t word_len, const void *data);
void write_tlb_reg(TTDevice *dev, uint32_t byte_addr, std::uint64_t value);

PCIdevice ttkmd_open(DWORD device_id, bool sharable /* = false */);
int ttkmd_close(struct PCIdevice &device);
std::vector<chip_id_t> ttkmd_scan();

void print_device_info (struct PCIdevice &d);
void set_debug_level(int dl);
void clr_printf(const char *clr, const char *fmt, ...);
