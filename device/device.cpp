// SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0

#include "device/device.h"

#include <spawn.h>
#include <wait.h>

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>
#include <memory>
#include <string_view>

#include "common/logger.hpp"
#include "device/architecture_implementation.h"
#include "device/driver_atomics.h"
#include "device/kmd.h"
#include "device/tlb.h"
#include "device/tt_silicon_driver_common.hpp"

using namespace std::literals;

// TODO: Should this be deleted? It is never set to true?!?
extern volatile bool msi_interrupt_received;

namespace tt::umd {

device::device(architecture arch, chip_id_t logical_device_id) : arch(arch), logical_device_id(logical_device_id) {
    arch_impl = architecture_implementation::create(arch);
}

pci_device::pci_device(std::unique_ptr<tt::umd::kmd> kmd, chip_id_t logical_device_id) :
    device(kmd->get_architecture(), logical_device_id), kmd(std::move(kmd)) {}

std::shared_ptr<device> pci_device::open(std::unique_ptr<tt::umd::kmd> kmd, chip_id_t logical_device_id) {
    std::shared_ptr<device> device{new pci_device(std::move(kmd), logical_device_id)};

    // TODO: Implement
    return std::move(device);
}

bool pci_device::address_in_tlb_space(uint32_t address, uint32_t size_in_bytes, int32_t tlb_index, uint32_t tlb_size) {
    auto config_iterator = tlb_config_map.find(tlb_index);

    return (config_iterator != tlb_config_map.end()) && (address >= config_iterator->second) &&
           (address + size_in_bytes <= config_iterator->second + tlb_size);
}

// TODO: Transfer logging -> move to new files?!?
// record_access, LOG1, LOG2, print_buffer

bool pci_device::is_hardware_hung() {
    volatile const void *address = reinterpret_cast<const uint8_t *>(kmd->get_bar0_uc()) + (arch_impl->get_arc_reset_scratch_offset() + 6 * 4) - kmd->get_bar0_uc_offset();
    uint32_t scratch_data = *reinterpret_cast<const volatile uint32_t*>(address);

    return (scratch_data == 0xffffffffu);
}

bool pci_device::reset_by_sysfs() {
    const char* virtual_env = getenv("VIRTUAL_ENV");
    if (virtual_env == nullptr)
        return false;

    std::string reset_helper_path = virtual_env;
    reset_helper_path += "/bin/reset-helper"sv;

    std::string bus_id = std::to_string(kmd->get_pci_bus());

    uint32_t kmd_index = kmd->get_index();
    kmd.reset();

    char* argv[3];
    argv[0] = const_cast<char*>(reset_helper_path.c_str());
    argv[1] = const_cast<char*>(bus_id.c_str());
    argv[2] = nullptr;

    pid_t reset_helper_pid;
    if (posix_spawn(&reset_helper_pid, reset_helper_path.c_str(), nullptr, nullptr, argv, environ) != 0) {
        return false;
    }

    siginfo_t reset_helper_status;
    if (waitid(P_PID, reset_helper_pid, &reset_helper_status, WEXITED) != 0) {
        return false;
    }

    if (reset_helper_status.si_status != 0) {
        return false;
    }

    kmd = tt::umd::kmd::open(kmd_index);
    return true;
}

bool pci_device::auto_reset_board() {
    return (kmd->reset_by_ioctl() || reset_by_sysfs()) && !is_hardware_hung();
}

void pci_device::detect_ffffffff_read(uint32_t data_read) {
    if (read_checking_enabled && data_read == 0xffffffffu && is_hardware_hung()) {
        if (auto_reset_board()) {
            throw std::runtime_error("Read 0xffffffff from ARC scratch[6]: auto-reset succeeded.");
        } else {
            throw std::runtime_error("Read 0xffffffff from ARC scratch[6]: you should reset the board.");
        }
    }
}

void pci_device::write_regs(uint32_t byte_address, uint32_t word_len, const void *data) {
    // record_access("write_regs", byte_address, word_len * sizeof(uint32_t), false, true, false, false);

    volatile uint32_t *dest = kmd->get_register_address<std::uint32_t>(byte_address);
    const uint32_t *src = reinterpret_cast<const uint32_t*>(data);

    while (word_len-- != 0) {
        uint32_t temp;
        memcpy(&temp, src++, sizeof(temp));
        *dest++ = temp;
    }
    // LOG2(" REG ");
    // print_buffer (data, std::min(g_NUM_BYTES_TO_PRINT, word_len * 4), true);
}

void pci_device::write_tlb_reg(uint32_t address, uint64_t value) {
    // record_access("write_tlb_reg", address, sizeof(value), false, true, false, false);

    volatile uint64_t *dest = kmd->get_register_address<uint64_t>(address);
#if defined(__ARM_ARCH) || defined(__riscv)
    // The store below goes through UC memory on x86, which has implicit ordering constraints with WC accesses.
    // ARM has no concept of UC memory. This will not allow for implicit ordering of this store wrt other memory accesses.
    // Insert an explicit full memory barrier for ARM.
    // Do the same for RISC-V.
    tt_driver_atomics::mfence();
#endif
    *dest = value;
    tt_driver_atomics::mfence(); // Otherwise subsequent WC loads move earlier than the above UC store to the TLB register.

    // LOG2(" TLB ");
    // print_buffer (&value, sizeof(value), true);
}

// Get TLB index (from zero), check if it's in 16MB, 2MB or 1MB TLB range, and dynamically program it.
dynamic_tlb pci_device::set_dynamic_tlb(
    uint32_t tlb_index, xy_pair start, xy_pair end, uint32_t address, bool multicast, tlb_data_ordering ordering) {
    if (multicast) {
        std::tie(start, end) = arch_impl->multicast_workaround(start, end);
    }

    // LOG2(
    //     "set_dynamic_tlb with arguments: tlb_index = %d, start = (%d, %d), end = (%d, %d), address = 0x%x, multicast
    //     = "
    //     "%d, ordering = %d\n",
    //     tlb_index,
    //     start.x,
    //     start.y,
    //     end.x,
    //     end.y,
    //     address,
    //     multicast,
    //     (int)ordering);

    tlb_configuration tlb_config = arch_impl->get_tlb_configuration(tlb_index);
    auto translated_start_coords = harvested_coord_translation.at(start);
    auto translated_end_coords = harvested_coord_translation.at(end);
    uint32_t tlb_address = address / tlb_config.size;
    uint32_t local_offset = address % tlb_config.size;
    uint32_t tlb_base = tlb_config.base + (tlb_config.size * tlb_config.index_offset);
    uint32_t tlb_cfg_reg = tlb_config.cfg_addr + (8 * tlb_config.index_offset);

    auto tlb_data_offset =
        tlb_data{
            .local_offset = tlb_address,
            .x_end = static_cast<uint64_t>(translated_end_coords.x),
            .y_end = static_cast<uint64_t>(translated_end_coords.y),
            .x_start = static_cast<uint64_t>(translated_start_coords.x),
            .y_start = static_cast<uint64_t>(translated_start_coords.y),
            .mcast = multicast,
            .ordering = static_cast<uint64_t>(ordering),
            .static_vc = true,
        }
            .apply_offset(tlb_config.offset);

    // LOG1(
    //     "set_dynamic_tlb() with tlb_index: %d tlb_index_offset: %d dynamic_tlb_size: %dMB tlb_base: 0x%x tlb_cfg_reg:
    //     " "0x%x\n", tlb_index, tlb_config.index_offset, tlb_config.size / (1024 * 1024), tlb_base, tlb_cfg_reg);
    write_tlb_reg(tlb_cfg_reg, *tlb_data_offset);
    return {tlb_base + local_offset, tlb_config.size - local_offset};
}

void pci_device::configure_tlb(xy_pair core, uint32_t tlb_index, uint32_t address, tlb_data_ordering ordering) {
    log_assert(
        ordering == tlb_data_ordering::strict || ordering == tlb_data_ordering::posted ||
            ordering == tlb_data_ordering::relaxed,
        "Invalid ordering specified in pci_device::configure_tlb");
    set_dynamic_tlb(tlb_index, core, address, ordering);
    auto tlb_size = std::get<1>(arch_impl->describe_tlb(tlb_index).value());
    tlb_config_map.insert({tlb_index, (address / tlb_size) * tlb_size});
}

uint32_t pci_device::pcie_dma_transfer_turbo(uint32_t chip_address, uint32_t host_physical_address, uint32_t size_bytes, bool write) {
    // c_timer t ("");

    // t.now_in ("1. DMA setup");

    if (csm_pcie_ctrl_dma_request_offset == 0) {
        throw std::runtime_error ("pcie_init_dma_transfer_turbo must be called before pcie_dma_transfer_turbo");
    }

    arc_pcie_ctrl_dma_request_t req = {
        .chip_addr           = chip_address,
        .host_phys_addr      = host_physical_address,
        .completion_flag_phys_addr = static_cast<uint32_t>(kmd->get_dma_completion_flag_buffer().get_physical_address()),
        .size_bytes          = size_bytes,
        .write               = (write ? 1U : 0U),
        .pcie_msi_on_done    = use_msi_for_dma ? 1U : 0U,
        .pcie_write_on_done  = use_msi_for_dma ? 0U : 1U,
        .trigger             = 1U,
        .repeat              = 1
    };

    volatile uint32_t *complete_flag = (uint32_t *)kmd->get_dma_completion_flag_buffer().get_user_address();
    *complete_flag = 0;

    // Configure the DMA engine
    msi_interrupt_received = false;
    write_regs(csm_pcie_ctrl_dma_request_offset, sizeof(req) / sizeof(uint32_t), &req);

    // Trigger ARC interrupt 0 on core 0
    int arc_misc_cntl_value = 0;

    // NOTE: Ideally, we should read the state of this register before writing to it, but that
    //       casues a lot of delay (reads have huge latencies)
    arc_misc_cntl_value |= (1 << 16); // Cause IRQ0 on core 0
    write_regs(arc_misc_cntl_address, 1, &arc_misc_cntl_value);

    if (!use_msi_for_dma) {
        // t.now_in ("2. DMA poll");
        int wait_loops = 0;
        while (true) {
            // The complete flag is set ty by ARC (see src/hardware/soc/tb/arc_fw/lib/pcie_dma.c)
            if (*complete_flag == 0xfaca) {
                break;
            }
            wait_loops++;
        }
        // LOG2 ("Waited %d iterations\n", wait_loops);
    } else {
        // t.now_in ("2. DMA wait for MSI");
        while (msi_interrupt_received == false) {}
    }

    return 0; // TODO: status
}

void pci_device::memcpy_from_device(void *destination, const void *source, std::size_t num_bytes) {
    using copy_t = uint32_t;

    // Start by aligning the source (device) pointer.
    const volatile copy_t *sp;

    uintptr_t source_address = reinterpret_cast<uintptr_t>(source);
    unsigned int src_misalignment = source_address % sizeof(copy_t);

    if (src_misalignment != 0) {
        sp = reinterpret_cast<copy_t*>(source_address - src_misalignment);

        copy_t tmp = *sp++;

        auto leading_len = std::min(sizeof(tmp) - src_misalignment, num_bytes);
        std::memcpy(destination, reinterpret_cast<char *>(&tmp) + src_misalignment, leading_len);
        num_bytes -= leading_len;
        destination = static_cast<uint8_t *>(destination) + leading_len;

    } else {
        sp = static_cast<const volatile copy_t*>(source);
    }

    // Copy the source-aligned middle.
    copy_t *dp = static_cast<copy_t *>(destination);
    size_t num_words = num_bytes / sizeof(copy_t);

    for (size_t i = 0; i < num_words; i++)
        *dp++ = *sp++;

    // Finally copy any sub-word trailer.
    auto trailing_len = num_bytes % sizeof(copy_t);
    if (trailing_len != 0) {
        copy_t tmp = *sp;
        std::memcpy(dp, &tmp, trailing_len);
    }
}

void pci_device::read_block(uint32_t address, uint32_t size_in_bytes, uint8_t* buffer_address) {
    if (size_in_bytes >= dma_block_size_read_threshold_bytes && dma_block_size_read_threshold_bytes > 0) {
        // record_access ("read_block_a", address, size_in_bytes, true, false, true, true); // addr, size, turbo, write, block, endline
        uint64_t host_physical_address = kmd->get_dma_transfer_buffer().get_physical_address();
        uint64_t host_user_address = kmd->get_dma_transfer_buffer().get_user_address();

        while (size_in_bytes > 0) {
            uint32_t transfered_bytes = std::min<uint32_t>(size_in_bytes, dma_buffer_size);
            pcie_dma_transfer_turbo (address, host_physical_address, transfered_bytes, false);
            memcpy (buffer_address, (void*)host_user_address, transfered_bytes);
            size_in_bytes -= transfered_bytes;
            address += transfered_bytes;
            buffer_address += transfered_bytes;
        }
        return;
    }

    // record_access("read_block_b", address, size_in_bytes, false, false, true, false); // addr, size, turbo, write, block, endline

    void *reg_mapping = kmd->get_reg_mapping_and_adjust_offset(address);
    const void *src = reinterpret_cast<const uint8_t *>(reg_mapping) + address;
    void *dest = reinterpret_cast<void *>(buffer_address);

#ifndef DISABLE_ISSUE_3487_FIX
    memcpy_from_device(dest, src, size_in_bytes);
#else
#ifdef FAST_MEMCPY

    if ((size_in_bytes % 32 == 0) && ((intptr_t(dest) & 31) == 0) && ((intptr_t(src) & 31) == 0))
    memcpy_from_device(dest, src, size_in_bytes);
    {
        // Faster memcpy version.. about 8x currently compared to pci_read above
        fastMemcpy(dest, src, size_in_bytes);
    }
    else
#else
    // ~4x faster than pci_read above, but works for all sizes and alignments
    memcpy(dest, src, size_in_bytes);
#endif
#endif

    if (size_in_bytes >= sizeof(std::uint32_t)) {
        detect_ffffffff_read(*reinterpret_cast<uint32_t*>(dest));
    }
    // print_buffer (buffer_address, std::min(g_NUM_BYTES_TO_PRINT, size_in_bytes), true);
}

void pci_device::read(void* memory, uint32_t size_in_bytes, xy_pair core, uint64_t address, const std::string& fallback_tlb) {
    if (fallback_tlb == "REG_TLB"sv) {
        read_registers(memory, size_in_bytes, core, address, fallback_tlb);
    } else {
        read_memory(memory, size_in_bytes, core, address, fallback_tlb);
    }
}

void pci_device::read_memory(void* memory, size_t size_in_bytes, xy_pair target, uint64_t address, const std::string& fallback_tlb) {
    // Assume that mem_ptr has been allocated adequate memory on host when this function is called. Otherwise, this
    // function will cause a segmentation fault. LOG1("---- pci_device::read_memory to chip:%lu %lu-%lu at 0x%x
    // size_in_bytes: %d\n", logical_device_id, target.x, target.y, address, size_in_bytes);

    uint8_t* buffer_address = static_cast<uint8_t*>(memory);

    std::int32_t tlb_index = 0;
    tlb_description tlb_data = std::nullopt;

    if (map_core_to_tlb) {
        tlb_index = map_core_to_tlb(target);
        tlb_data = arch_impl->describe_tlb(tlb_index);
    }
    // LOG1("  tlb_index: %d, tlb_data.has_value(): %d\n", tlb_index, tlb_data.has_value());

    if (tlb_data.has_value() &&
        address_in_tlb_space(address, size_in_bytes, tlb_index, std::get<1>(tlb_data.value()))) {
        auto [tlb_offset, tlb_size] = tlb_data.value();
        read_block(tlb_offset + address % tlb_size, size_in_bytes, buffer_address);
        // LOG1 ("  read_block called with tlb_offset: %d, tlb_size: %d\n", tlb_offset, tlb_size);
    } else {
        // TODO: Continue
        // const auto tlb_index = dynamic_tlb_config.at(fallback_tlb);
        // const boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*get_mutex(fallback_tlb, pci_device->id));
        // LOG1("  dynamic tlb_index: %d\n", tlb_index);
        // while (size_in_bytes > 0) {
        //     auto [mapped_address, tlb_size] = set_dynamic_tlb(
        //         pci_device,
        //         architecture_implementation.get(),
        //         tlb_index,
        //         target,
        //         address,
        //         harvested_coord_translation,
        //         dynamic_tlb_ordering_modes.at(fallback_tlb));
        //     uint32_t transfer_size = std::min(size_in_bytes, tlb_size);
        //     read_block(
        //         dev, architecture_implementation.get(), mapped_address, transfer_size, buffer_addr, m_dma_buf_size);

        //     size_in_bytes -= transfer_size;
        //     address += transfer_size;
        //     buffer_addr += transfer_size;
        // }
        // LOG1 ("Read done Dynamic TLB with pid=%ld\n", (long)getpid());
    }
}

void pci_device::read_registers(void* memory, size_t size, xy_pair core, uint64_t address, const std::string& fallback_tlb) {
    // read_mmio_device_register(mem_ptr, core, addr, size, fallback_tlb);
}

}  // namespace tt::umd
