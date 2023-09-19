#include "pci_comms.h"

#include <spawn.h>
#include <memory>
#include <sys/ioctl.h>
#include <linux/pci.h>
#include <stdio.h>
#include <iostream>
#include <cstdint>
#include <string.h>
#include <cstring>
#include <immintrin.h>
#include <unistd.h>
#include <string>

#include <utility>
#include <cstdarg>
#include <cstdio>
#include <fstream>
#include <fstream>

#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <dirent.h>
#include <fcntl.h>
#include <unistd.h>
#include <spawn.h>
#include <wait.h>
#include <errno.h>
#include <immintrin.h>
#include <assert.h>
#include <unordered_map>

#include "ioctl.h"
#include "tt_cluster_descriptor_types.h"
#include "impl_device.hpp"
#include "device_data.hpp"

#include "common/logger.hpp"

#define WHT "\e[0;37m"
#define BLK "\e[0;30m"
#define RED "\e[0;31m"
#define GRN "\e[0;32m"
#define YEL "\e[0;33m"
#define BLU "\e[0;34m"
#define RST "\e[0m"
#define LOG1(...) if (g_DEBUG_LEVEL > 0) clr_printf("", __VA_ARGS__)  // Mostly debugging
#define LOG2(...) if (g_DEBUG_LEVEL > 1) clr_printf("", __VA_ARGS__)  // Mostly debugging
#define PRINT(...) clr_printf("",__VA_ARGS__)                       // What users should see
// #define LOG(...) if (false) clr_printf("", __VA_ARGS__)   // Mostly debugging
// #define PRINT(...) if (false) clr_printf(BLK, __VA_ARGS__)                       // What users should see
#define WARN(...)  clr_printf(YEL, __VA_ARGS__)                       // Something wrong
#define ERROR(...) clr_printf(RED, __VA_ARGS__)                       // Something very bad

void clr_printf(const char *clr, const char *fmt, ...) {
    va_list args; va_start(args, fmt); printf ("%s", clr); vprintf(fmt, args); printf (RST); va_end(args);
}

int g_DEBUG_LEVEL; // /src/t6ifc/t6py/packages/tenstorrent/jlink/jtag_comm.cpp
bool g_READ_CHECKING_ENABLED = true;

bool g_USE_MSI_FOR_DMA = false; // Whether to wait for MSI after DMA transfer, or poll a variable
uint32_t g_DMA_BLOCK_SIZE_READ_THRESHOLD_BYTES = 0;  // 0 - never use DMA. Otherwise use DMA for all blocks larger than this size
uint32_t g_DMA_BLOCK_SIZE_WRITE_THRESHOLD_BYTES = 0; // 0 - never use DMA. Otherwise use DMA for all blocks larger than this size

// Address in CSM where the DMA request structure resides
uint32_t c_CSM_PCIE_CTRL_DMA_REQUEST_OFFSET = 0;
// Address where the trigger for transfer resides
uint32_t c_DMA_TRIGGER_ADDRESS = 0;
// To trigger arc interrupt
uint32_t c_ARC_MISC_CNTL_ADDRESS = 0;

// Print all buffers smaller than this number of bytes
uint32_t g_NUM_BYTES_TO_PRINT = 8;

volatile bool msi_interrupt_received = false;

const char device_name_pattern[] = "/dev/tenstorrent/%u";

static uint32_t GS_BAR0_WC_MAPPING_SIZE = (156<<20) + (10<<21) + (18<<24);

uint32_t pcie_dma_transfer_turbo (TTDevice *dev, uint32_t chip_addr, uint32_t host_phys_addr, uint32_t size_bytes, bool write);

bool is_grayskull(const uint16_t device_id) {
    return device_id == 0xfaca;
}

bool is_wormhole(const uint16_t device_id) {
    return device_id == 0x401e;
}

bool is_wormhole(const tenstorrent_get_device_info_out &device_info) {
    return is_wormhole(device_info.device_id);
}

bool is_wormhole_b0(const uint16_t device_id, const uint16_t revision_id) {
    return (is_wormhole(device_id) && (revision_id == 0x01));
}

int find_device(const uint16_t device_id) {
    // returns device id if found, otherwise -1
    char device_name[sizeof(device_name_pattern) + std::numeric_limits<unsigned int>::digits10];
    std::snprintf(device_name, sizeof(device_name), device_name_pattern, (unsigned int)device_id);
    int device_fd = ::open(device_name, O_RDWR | O_CLOEXEC);
    LOG2 ("find_device() open call returns device_fd: %d for device_name: %s (device_id: %d)\n", device_fd, device_name, device_id);
    return device_fd;
}

// Open a unique device_id per host memory channel (workaround for ttkmd < 1.21 support for more than 1 pin per fd)
void TTDevice::open_hugepage_per_host_mem_ch(uint32_t num_host_mem_channels) {
    for (int ch = 0; ch < num_host_mem_channels; ch++) {
        tt_device_logger::log_debug(tt_device_logger::LogSiliconDriver, "Opening device_fd_per_host_ch device index: {} ch: {} (num_host_mem_channels: {})", index, ch, num_host_mem_channels);
        int device_fd_for_host_mem = find_device(index);
        if (device_fd_for_host_mem == -1) {
            throw std::runtime_error(std::string("Failed opening a host memory device handle for device ") + std::to_string(index));
        }
        device_fd_per_host_ch.push_back(device_fd_for_host_mem);
    }
}

TTDevice TTDevice::open(unsigned int device_id) {
    TTDevice ttdev;
    static int unique_id = 0;
    ttdev.index = device_id;
    ttdev.do_open();

    return ttdev;
}

void TTDevice::do_open() {
    device_fd = find_device(index);
    if (device_fd == -1) {
        throw std::runtime_error(std::string("Failed opening a handle for device ") + std::to_string(index));
    }

    tenstorrent_get_device_info device_info;
    memset(&device_info, 0, sizeof(device_info));
    device_info.in.output_size_bytes = sizeof(device_info.out);

    if (ioctl(device_fd, TENSTORRENT_IOCTL_GET_DEVICE_INFO, &device_info) == -1) {
        throw std::runtime_error(std::string("Get device info failed on device ") + std::to_string(index) + ".");
    }

    this->device_info = device_info.out;

    max_dma_buf_size_log2 = device_info.out.max_dma_buf_size_log2;

    struct {
        tenstorrent_query_mappings query_mappings;
        tenstorrent_mapping mapping_array[8];
    } mappings;

    memset(&mappings, 0, sizeof(mappings));
    mappings.query_mappings.in.output_mapping_count = 8;

    if (ioctl(device_fd, TENSTORRENT_IOCTL_QUERY_MAPPINGS, &mappings.query_mappings) == -1) {
        throw std::runtime_error(std::string("Query mappings failed on device ") + std::to_string(index) + ".");
    }

    tenstorrent_mapping bar0_uc_mapping;
    tenstorrent_mapping bar0_wc_mapping;
    tenstorrent_mapping bar2_uc_mapping;
    tenstorrent_mapping bar2_wc_mapping;

    memset(&bar0_uc_mapping, 0, sizeof(bar0_uc_mapping));
    memset(&bar0_wc_mapping, 0, sizeof(bar0_wc_mapping));
    memset(&bar2_uc_mapping, 0, sizeof(bar2_uc_mapping));
    memset(&bar2_wc_mapping, 0, sizeof(bar2_wc_mapping));

    for (unsigned int i = 0; i < mappings.query_mappings.in.output_mapping_count; i++) {
        if (mappings.mapping_array[i].mapping_id == TENSTORRENT_MAPPING_RESOURCE0_UC) {
            bar0_uc_mapping = mappings.mapping_array[i];
        }

        if (mappings.mapping_array[i].mapping_id == TENSTORRENT_MAPPING_RESOURCE0_WC) {
            bar0_wc_mapping = mappings.mapping_array[i];
        }

        if (mappings.mapping_array[i].mapping_id == TENSTORRENT_MAPPING_RESOURCE2_UC) {
            bar2_uc_mapping = mappings.mapping_array[i];
        }

        if (mappings.mapping_array[i].mapping_id == TENSTORRENT_MAPPING_RESOURCE2_WC) {
            bar2_wc_mapping = mappings.mapping_array[i];
        }
    }

    if (bar0_uc_mapping.mapping_id != TENSTORRENT_MAPPING_RESOURCE0_UC) {
        throw std::runtime_error(std::string("Device ") + std::to_string(index) + " has no BAR0 UC mapping.");
    }

    // Attempt WC mapping first so we can fall back to all-UC if it fails.
    if (bar0_wc_mapping.mapping_id == TENSTORRENT_MAPPING_RESOURCE0_WC) {
        bar0_wc_size = std::min<size_t>(bar0_wc_mapping.mapping_size, GS_BAR0_WC_MAPPING_SIZE);
        bar0_wc = mmap(NULL, bar0_wc_size, PROT_READ | PROT_WRITE, MAP_SHARED, device_fd, bar0_wc_mapping.mapping_base);
        if (bar0_wc == MAP_FAILED) {
            bar0_wc_size = 0;
            bar0_wc = nullptr;
        }
    }

    if (bar0_wc) {
        // The bottom part of the BAR is mapped WC. Map the top UC.
        bar0_uc_size = bar0_uc_mapping.mapping_size - GS_BAR0_WC_MAPPING_SIZE;
        bar0_uc_offset = GS_BAR0_WC_MAPPING_SIZE;
    } else {
        // No WC mapping, map the entire BAR UC.
        bar0_uc_size = bar0_uc_mapping.mapping_size;
        bar0_uc_offset = 0;
    }

    bar0_uc = mmap(NULL, bar0_uc_size, PROT_READ | PROT_WRITE, MAP_SHARED, device_fd, bar0_uc_mapping.mapping_base + bar0_uc_offset);

    if (bar0_uc == MAP_FAILED) {
        throw std::runtime_error(std::string("BAR0 UC memory mapping failed for device ") + std::to_string(index) + ".");
    }

    if (!bar0_wc) {
        bar0_wc = bar0_uc;
    }

    if (is_wormhole(device_info.out)) {
        if (bar2_uc_mapping.mapping_id != TENSTORRENT_MAPPING_RESOURCE2_UC) {
            throw std::runtime_error(std::string("Device ") + std::to_string(index) + " has no BAR4 UC mapping.");
        }

        this->system_reg_mapping_size = bar2_uc_mapping.mapping_size;

        this->system_reg_mapping = mmap(NULL, bar2_uc_mapping.mapping_size, PROT_READ | PROT_WRITE, MAP_SHARED, device_fd, bar2_uc_mapping.mapping_base);

        if (this->system_reg_mapping == MAP_FAILED) {
            throw std::runtime_error(std::string("BAR4 UC memory mapping failed for device ") + std::to_string(index) + ".");
        }

        this->system_reg_start_offset = (512 - 16) * 1024*1024;
        this->system_reg_offset_adjust = (512 - 32) * 1024*1024;
    }

    pci_bus = device_info.out.bus_dev_fn >> 8;
    pci_device = PCI_SLOT(device_info.out.bus_dev_fn);
    pci_function = PCI_FUNC(device_info.out.bus_dev_fn);
}

void set_debug_level(int dl) {
    g_DEBUG_LEVEL = dl;
}

std::uint64_t pci_dma_buffer_get_physical_addr(DMAbuffer &dma_buffer) {
    assert (dma_buffer.pDma);
    return reinterpret_cast<std::uint64_t>(dma_buffer.pDma);
}

std::uint64_t pci_dma_buffer_get_user_addr(DMAbuffer &dma_buffer) {
    assert (dma_buffer.pBuf);
    return reinterpret_cast<std::uint64_t>(dma_buffer.pBuf);
}

DWORD ttkmd_init() { return 0; }    // 0 on success
DWORD ttkmd_uninit() { return 0; }  // 0 on success

bool is_char_dev(const dirent *ent, const char *parent_dir) {
    if (ent->d_type == DT_UNKNOWN || ent->d_type == DT_LNK) {
        char name[2 * NAME_MAX + 2];
        strcpy(name, parent_dir);
        strcat(name, "/");
        strcat(name, ent->d_name);

        struct stat stat_result;
        if (stat(name, &stat_result) == -1) {
            return false;
        }

        return ((stat_result.st_mode & S_IFMT) == S_IFCHR);
    } else {
        return (ent->d_type == DT_CHR);
    }
}

std::vector<chip_id_t> ttkmd_scan() {

    static const char dev_dir[] = "/dev/tenstorrent";

    std::vector<chip_id_t> found_devices;

    DIR *d = opendir(dev_dir);
    if (d != nullptr) {
        while (true) {
            const dirent *ent = readdir(d);
            if (ent == nullptr) {
                break;
            }

            // strtoul allows initial whitespace, +, -
            if (!std::isdigit(ent->d_name[0])) {
                continue;
            }

            if (!is_char_dev(ent, dev_dir)) {
                continue;
            }

            char *endptr = nullptr;
            errno = 0;
            unsigned long index = std::strtoul(ent->d_name, &endptr, 10);
            if (index == std::numeric_limits<unsigned int>::max() && errno == ERANGE) {
                continue;
            }
            if (*endptr != '\0') {
                continue;
            }

            found_devices.push_back( (chip_id_t) index);
        }
        closedir(d);
    }

    std::sort(found_devices.begin(), found_devices.end());

    return found_devices;
}

int get_config_space_fd(TTDevice *dev) {
    if (dev->sysfs_config_fd == -1) {
        static const char pattern[] = "/sys/bus/pci/devices/0000:%02x:%02x.%u/config";
        char buf[sizeof(pattern)];
        std::snprintf(buf, sizeof(buf), pattern,
                      (unsigned int)dev->pci_bus, (unsigned int)dev->pci_device, (unsigned int)dev->pci_function);
        dev->sysfs_config_fd = open(buf, O_RDWR);

        if (dev->sysfs_config_fd == -1) {
            dev->sysfs_config_fd = open(buf, O_RDONLY);
        }
    }

    return dev->sysfs_config_fd;
}

int get_revision_id(TTDevice *dev) {

    static const char pattern[] = "/sys/bus/pci/devices/0000:%02x:%02x.%u/revision";
    char buf[sizeof(pattern)];
    std::snprintf(buf, sizeof(buf), pattern,
    (unsigned int)dev->pci_bus, (unsigned int)dev->pci_device, (unsigned int)dev->pci_function);

    std::ifstream revision_file(buf);
    std::string revision_string;
    if (std::getline(revision_file, revision_string)) {
        return std::stoi(revision_string, nullptr, 0);
    } else {
        throw std::runtime_error("Revision ID read failed for device");
    }
}

std::uint64_t read_bar0_base(TTDevice *dev) {
    const std::uint64_t bar_address_mask = ~(std::uint64_t)0xF;
    unsigned int bar0_config_offset = 0x10;

    std::uint64_t bar01;
    if (pread(get_config_space_fd(dev), &bar01, sizeof(bar01), bar0_config_offset) != sizeof(bar01)) {
        return 0;
    }

    return bar01 & bar_address_mask;
}

DMAbuffer allocate_dma_buffer(TTDevice *ttdev, unsigned int buffer_index, std::size_t size) {
    tenstorrent_allocate_dma_buf allocate_dma_buf;

    if (size > std::numeric_limits<decltype(allocate_dma_buf.in.requested_size)>::max()) {
        throw std::runtime_error(std::string("Requested DMA buffer size (" + std::to_string(allocate_dma_buf.in.requested_size)
                                             + ") bytes exceeds interface size limit for device " + std::to_string(ttdev->index) + ", with error: " + std::strerror(errno)));
    }

    memset(&allocate_dma_buf, 0, sizeof(allocate_dma_buf));
    allocate_dma_buf.in.requested_size = std::max<std::size_t>(size, getpagesize());
    allocate_dma_buf.in.buf_index = buffer_index;

    if (ioctl(ttdev->device_fd, TENSTORRENT_IOCTL_ALLOCATE_DMA_BUF, &allocate_dma_buf) == -1) {
        throw std::runtime_error(std::string("DMA buffer allocation failed (") + std::to_string(allocate_dma_buf.in.requested_size)
                                 + " bytes) for device " + std::to_string(ttdev->index) + ".");
    }

    void *mapping = mmap(NULL, allocate_dma_buf.out.size, PROT_READ | PROT_WRITE, MAP_SHARED, ttdev->device_fd, allocate_dma_buf.out.mapping_offset);

    std::cout << "DMA buffer succeeded with size " << allocate_dma_buf.out.size << " offset " << allocate_dma_buf.out.mapping_offset << " phy_addr " << allocate_dma_buf.out.physical_address << std::endl;

    if (mapping == MAP_FAILED) {
        throw std::runtime_error(std::string("DMA buffer memory mapping failed for device ") + std::to_string(ttdev->index) + ".");
    }

    DMAbuffer dmabuf;
    dmabuf.pBuf = mapping;
    dmabuf.pDma = allocate_dma_buf.out.physical_address;
    dmabuf.size = allocate_dma_buf.out.size;

    ttdev->dma_buffer_mappings.push_back(dmabuf);

    return dmabuf;
}

PCIdevice ttkmd_open(DWORD device_id, bool sharable /* = false */)
{
    (void)sharable; // presently ignored

    auto ttdev = std::make_unique<TTDevice>(TTDevice::open(device_id));

    PCIdevice device;
    device.id = device_id;
    device.hdev = ttdev.get();
    device.vendor_id = ttdev->device_info.vendor_id;
    device.device_id = ttdev->device_info.device_id;
    device.subsystem_vendor_id = ttdev->device_info.subsystem_vendor_id;
    device.subsystem_id = ttdev->device_info.subsystem_id;
    device.dwBus = ttdev->pci_bus;
    device.dwSlot = ttdev->pci_device;
    device.dwFunction = ttdev->pci_function;
    device.BAR_addr = read_bar0_base(ttdev.get());
    device.BAR_size_bytes = ttdev->bar0_uc_size;
    device.revision_id = get_revision_id(ttdev.get());
    ttdev.release();

    return device;
}

int ttkmd_close(struct PCIdevice &device) {
    delete static_cast<TTDevice*>(device.hdev);

    return 0;
}

template <class T>
volatile T* register_address(const TTDevice *dev, std::uint32_t register_offset) {
    void *reg_mapping;
    if (dev->system_reg_mapping != nullptr && register_offset >= dev->system_reg_start_offset) {
        register_offset -= dev->system_reg_offset_adjust;
        reg_mapping = dev->system_reg_mapping;
    } else if (dev->bar0_wc != dev->bar0_uc && register_offset < dev->bar0_wc_size) {
        reg_mapping = dev->bar0_wc;
    } else {
        register_offset -= dev->bar0_uc_offset;
        reg_mapping = dev->bar0_uc;
    }

    return reinterpret_cast<T*>(static_cast<uint8_t*>(reg_mapping) + register_offset);
}

bool is_hardware_hung(const TTDevice *dev) {
    volatile const void *addr = reinterpret_cast<const char *>(dev->bar0_uc) + (DEVICE_DATA.ARC_RESET_SCRATCH_OFFSET + 6 * 4) - dev->bar0_uc_offset;
    std::uint32_t scratch_data = *reinterpret_cast<const volatile std::uint32_t*>(addr);

    return (scratch_data == 0xffffffffu);
}

bool reset_by_sysfs(TTDevice *dev) {

    const char *virtual_env = getenv("VIRTUAL_ENV");
    if (virtual_env == nullptr)
        return false;

    std::string reset_helper_path = virtual_env;
    reset_helper_path += "/bin/reset-helper";

    std::string busid = std::to_string(dev->pci_bus);

    dev->suspend_before_device_reset();

    char *argv[3];
    argv[0] = const_cast<char*>(reset_helper_path.c_str());
    argv[1] = const_cast<char*>(busid.c_str());
    argv[2] = nullptr;

    pid_t reset_helper_pid;
    if (posix_spawn(&reset_helper_pid, reset_helper_path.c_str(), nullptr, nullptr, argv, environ) != 0)
        return false;

    siginfo_t reset_helper_status;
    if (waitid(P_PID, reset_helper_pid, &reset_helper_status, WEXITED) != 0)
        return false;

    if (reset_helper_status.si_status != 0)
        return false;

    dev->resume_after_device_reset();

    return true;
}

bool reset_by_ioctl(TTDevice *dev) {
    struct tenstorrent_reset_device reset_device;
    memset(&reset_device, 0, sizeof(reset_device));

    reset_device.in.output_size_bytes = sizeof(reset_device.out);
    reset_device.in.flags = 0;

    if (ioctl(dev->device_fd, TENSTORRENT_IOCTL_RESET_DEVICE, &reset_device) == -1) {
        return false;
    }

    return (reset_device.out.result == 0);
}

bool auto_reset_board(TTDevice *dev) {
    return ((reset_by_ioctl(dev) || reset_by_sysfs(dev)) && !is_hardware_hung(dev));
}

void detect_ffffffff_read(TTDevice *dev, std::uint32_t data_read = 0xffffffffu) {
    if (g_READ_CHECKING_ENABLED && data_read == 0xffffffffu && is_hardware_hung(dev)) {
        if (auto_reset_board(dev)) {
            throw std::runtime_error("Read 0xffffffff from ARC scratch[6]: auto-reset succeeded.");
        } else {
            throw std::runtime_error("Read 0xffffffff from ARC scratch[6]: you should reset the board.");
        }
    }
}

inline void record_access (const char* where, uint32_t addr, uint32_t size, bool turbo, bool write, bool block, bool endline) {
    LOG2 ("%s PCI_ACCESS %s 0x%8x  %8d bytes %s %s%s", where, write ? "WR" : "RD", addr, size, turbo ? "TU" : "  ", block ? "BLK" : "   ", endline ? "\n" : "" );
}

inline void print_buffer (uint64_t buffer_addr, uint32_t len_bytes = 16, bool endline = true) {
    if (g_DEBUG_LEVEL > 1) {
        uint32_t *b = (uint32_t *)(buffer_addr);
        for (uint32_t i = 0; i < len_bytes / 4; i++) {
            LOG2 ("    [0x%x] = 0x%x (%u) ", i, b[i], b[i]);
        }
        if (endline) {
            LOG2 ("\n");
        }
    }
}

// Get TLB index (from zero), check if it's in 16MB, 2MB or 1MB TLB range, and dynamically program it.
dynamic_tlb set_dynamic_tlb(PCIdevice* dev, unsigned int tlb_index, tt_xy_pair start, tt_xy_pair end,
                            std::uint32_t address, bool multicast, std::unordered_map<chip_id_t, std::unordered_map<tt_xy_pair, tt_xy_pair>>& harvested_coord_translation, bool posted) {

    LOG2("set_dynamic_tlb with arguments: tlb_index = %d, start = (%d, %d), end = (%d, %d), address = 0x%x, multicast = %d, posted = %d\n",
         tlb_index, start.x, start.y, end.x, end.y, address, multicast, posted);

    uint32_t dynamic_tlb_size, dynamic_tlb_base, dynamic_tlb_cfg_addr, tlb_index_offset;
    TLB_OFFSETS tlb_offset;
    auto translated_start_coords = harvested_coord_translation.at(dev -> logical_id).at(start);
    auto translated_end_coords = harvested_coord_translation.at(dev -> logical_id).at(end);

    if (tlb_index >= DEVICE_DATA.TLB_BASE_INDEX_16M)
    {
        dynamic_tlb_size        = DEVICE_DATA.DYNAMIC_TLB_16M_SIZE;
        dynamic_tlb_base        = DEVICE_DATA.DYNAMIC_TLB_16M_BASE;
        dynamic_tlb_cfg_addr    = DEVICE_DATA.DYNAMIC_TLB_16M_CFG_ADDR;
        tlb_offset              = TLB_16M_OFFSET;
        tlb_index_offset        = tlb_index - DEVICE_DATA.TLB_BASE_INDEX_16M;
    }
    else if (tlb_index >= DEVICE_DATA.TLB_BASE_INDEX_2M)
    {
        dynamic_tlb_size        = DEVICE_DATA.DYNAMIC_TLB_2M_SIZE;
        dynamic_tlb_base        = DEVICE_DATA.DYNAMIC_TLB_2M_BASE;
        dynamic_tlb_cfg_addr    = DEVICE_DATA.DYNAMIC_TLB_2M_CFG_ADDR;
        tlb_offset              = TLB_2M_OFFSET;
        tlb_index_offset        = tlb_index - DEVICE_DATA.TLB_BASE_INDEX_2M;
    }
    else
    {
        dynamic_tlb_size        = DEVICE_DATA.DYNAMIC_TLB_1M_SIZE;
        dynamic_tlb_base        = DEVICE_DATA.DYNAMIC_TLB_1M_BASE;
        dynamic_tlb_cfg_addr    = DEVICE_DATA.DYNAMIC_TLB_1M_CFG_ADDR;
        tlb_offset              = TLB_1M_OFFSET;
        tlb_index_offset        = tlb_index - DEVICE_DATA.TLB_BASE_INDEX_1M;
    }

    uint32_t tlb_address    = address / dynamic_tlb_size;
    uint32_t local_offset   = address % dynamic_tlb_size;
    uint32_t tlb_base       = dynamic_tlb_base + (dynamic_tlb_size * tlb_index_offset);
    uint32_t tlb_cfg_reg    = dynamic_tlb_cfg_addr + (8 * tlb_index_offset);

    auto tlb_data = TLB_DATA {
        .local_offset = tlb_address,
        .x_end = static_cast<uint64_t>(translated_end_coords.x),
        .y_end = static_cast<uint64_t>(translated_end_coords.y),
        .x_start = static_cast<uint64_t>(translated_start_coords.x),
        .y_start = static_cast<uint64_t>(translated_start_coords.y),
        .mcast = multicast,
        .ordering = posted ? TLB_DATA::Posted : TLB_DATA::Relaxed,
        .static_vc = true,
    }.apply_offset(tlb_offset);

    LOG1 ("set_dynamic_tlb() with tlb_index: %d tlb_index_offset: %d dynamic_tlb_size: %dMB tlb_base: 0x%x tlb_cfg_reg: 0x%x\n", tlb_index, tlb_index_offset, dynamic_tlb_size/(1024*1024), tlb_base, tlb_cfg_reg);
    //write_regs(dev -> hdev, tlb_cfg_reg, 2, &tlb_data);
    write_tlb_reg(dev->hdev, tlb_cfg_reg, *tlb_data);

    return { tlb_base + local_offset, dynamic_tlb_size - local_offset };
}

dynamic_tlb set_dynamic_tlb(PCIdevice *dev, unsigned int tlb_index, tt_xy_pair target, std::uint32_t address, std::unordered_map<chip_id_t, std::unordered_map<tt_xy_pair, tt_xy_pair>>& harvested_coord_translation, bool posted) {
    return set_dynamic_tlb(dev, tlb_index, tt_xy_pair(0, 0), target, address, false, harvested_coord_translation, posted);
}

dynamic_tlb set_dynamic_tlb_broadcast(PCIdevice *dev, unsigned int tlb_index, std::uint32_t address, std::unordered_map<chip_id_t, std::unordered_map<tt_xy_pair, tt_xy_pair>>& harvested_coord_translation, uint32_t num_rows_harvested, bool posted) {
    // When we have HW harvesting performed on WH, do not broadcast write to lower logical rows (or the physically harvested rows), as this hangs device during boot
    return set_dynamic_tlb (dev, tlb_index, tt_xy_pair(0, 0), tt_xy_pair(DEVICE_DATA.GRID_SIZE_X - 1, DEVICE_DATA.GRID_SIZE_Y - 1 - num_rows_harvested),
                            address, true, harvested_coord_translation, posted);
}

// Custom device memcpy. This is only safe for memory-like regions on the device (Tensix L1, DRAM, ARC CSM).
// Both routines assume that misaligned accesses are permitted on host memory.
//
// 1. AARCH64 device memory does not allow unaligned accesses (including pair loads/stores),
// which glibc's memcpy may perform when unrolling. This affects from and to device.
// 2. syseng#3487 WH GDDR5 controller has a bug when 1-byte writes are temporarily adjacent
// to 2-byte writes. We avoid ever performing a 1-byte write to the device. This only affects to device.

void memcpy_to_device(void *dest, const void *src, std::size_t num_bytes) {
    typedef std::uint32_t copy_t;

    // Start by aligning the destination (device) pointer. If needed, do RMW to fix up the
    // first partial word.
    volatile copy_t *dp;

    std::uintptr_t dest_addr = reinterpret_cast<std::uintptr_t>(dest);
    unsigned int dest_misalignment = dest_addr % sizeof(copy_t);

    if (dest_misalignment != 0) {
        // Read-modify-write for the first dest element.
        dp = reinterpret_cast<copy_t*>(dest_addr - dest_misalignment);

        copy_t tmp = *dp;

        auto leading_len = std::min(sizeof(tmp) - dest_misalignment, num_bytes);

        std::memcpy(reinterpret_cast<char*>(&tmp) + dest_misalignment, src, leading_len);
        num_bytes -= leading_len;
        src = static_cast<const char *>(src) + leading_len;

        *dp++ = tmp;

    } else {
        dp = static_cast<copy_t*>(dest);
    }

    // Copy the destination-aligned middle.
    const copy_t *sp = static_cast<const copy_t*>(src);
    std::size_t num_words = num_bytes / sizeof(copy_t);

    for (std::size_t i = 0; i < num_words; i++)
        *dp++ = *sp++;

    // Finally copy any sub-word trailer, again RMW on the destination.
    auto trailing_len = num_bytes % sizeof(copy_t);
    if (trailing_len != 0) {
        copy_t tmp = *dp;

        std::memcpy(&tmp, sp, trailing_len);

        *dp++ = tmp;
    }
}

void memcpy_from_device(void *dest, const void *src, std::size_t num_bytes) {
    typedef std::uint32_t copy_t;

    // Start by aligning the source (device) pointer.
    const volatile copy_t *sp;

    std::uintptr_t src_addr = reinterpret_cast<std::uintptr_t>(src);
    unsigned int src_misalignment = src_addr % sizeof(copy_t);

    if (src_misalignment != 0) {
        sp = reinterpret_cast<copy_t*>(src_addr - src_misalignment);

        copy_t tmp = *sp++;

        auto leading_len = std::min(sizeof(tmp) - src_misalignment, num_bytes);
        std::memcpy(dest, reinterpret_cast<char *>(&tmp) + src_misalignment, leading_len);
        num_bytes -= leading_len;
        dest = static_cast<char *>(dest) + leading_len;

    } else {
        sp = static_cast<const volatile copy_t*>(src);
    }

    // Copy the source-aligned middle.
    copy_t *dp = static_cast<copy_t *>(dest);
    std::size_t num_words = num_bytes / sizeof(copy_t);

    for (std::size_t i = 0; i < num_words; i++)
        *dp++ = *sp++;

    // Finally copy any sub-word trailer.
    auto trailing_len = num_bytes % sizeof(copy_t);
    if (trailing_len != 0) {
        copy_t tmp = *sp;
        std::memcpy(dp, &tmp, trailing_len);
    }
}

void read_block(TTDevice *dev, uint32_t byte_addr, uint32_t num_bytes, uint64_t buffer_addr, uint32_t dma_buf_size) {
    if (num_bytes >= g_DMA_BLOCK_SIZE_READ_THRESHOLD_BYTES && g_DMA_BLOCK_SIZE_READ_THRESHOLD_BYTES > 0) {
        record_access ("read_block_a", byte_addr, num_bytes, true, false, true, true); // addr, size, turbo, write, block, endline

        DMAbuffer &transfer_buffer = dev->dma_transfer_buffer;

        uint64_t host_phys_addr = pci_dma_buffer_get_physical_addr (transfer_buffer);
        uint64_t host_user_addr = pci_dma_buffer_get_user_addr (transfer_buffer);
        while (num_bytes > 0) {
            uint32_t transfered_bytes = std::min<uint32_t>(num_bytes, dma_buf_size);
            pcie_dma_transfer_turbo (dev, byte_addr, host_phys_addr, transfered_bytes, false);
            memcpy ((void*)buffer_addr, (void*)host_user_addr, transfered_bytes);
            num_bytes -= transfered_bytes;
            byte_addr += transfered_bytes;
            buffer_addr += transfered_bytes;
        }
        return;
    }

    record_access("read_block_b", byte_addr, num_bytes, false, false, true, false); // addr, size, turbo, write, block, endline

    auto reg_mapping = (char *)register_address<char>(dev, byte_addr);

    const void *src = reinterpret_cast<const void *>(reg_mapping);
    void *dest = reinterpret_cast<void *>(buffer_addr);

#ifndef DISABLE_ISSUE_3487_FIX
    memcpy_from_device(dest, src, num_bytes);
#else
#ifdef FAST_MEMCPY

    if ((num_bytes % 32 == 0) && ((intptr_t(dest) & 31) == 0) && ((intptr_t(src) & 31) == 0))
    memcpy_from_device(dest, src, num_bytes);
    {
        // Faster memcpy version.. about 8x currently compared to pci_read above
        fastMemcpy(dest, src, num_bytes);
    }
    else
#else
    // ~4x faster than pci_read above, but works for all sizes and alignments
    memcpy(dest, src, num_bytes);
#endif
#endif

    if (num_bytes >= sizeof(std::uint32_t)) {
        detect_ffffffff_read(dev, *reinterpret_cast<std::uint32_t*>(dest));
    }
    print_buffer (buffer_addr, std::min(g_NUM_BYTES_TO_PRINT, num_bytes), true);
}

void write_block(TTDevice *dev, uint32_t byte_addr, uint32_t num_bytes, uint64_t buffer_addr, uint32_t dma_buf_size) {
    if (num_bytes >= g_DMA_BLOCK_SIZE_WRITE_THRESHOLD_BYTES && g_DMA_BLOCK_SIZE_WRITE_THRESHOLD_BYTES > 0) {
        record_access ("write_block_a", byte_addr, num_bytes, true, true, true, true); // addr, size, turbo, write, block, endline

        DMAbuffer &transfer_buffer = dev->dma_transfer_buffer;

        uint64_t host_phys_addr = pci_dma_buffer_get_physical_addr (transfer_buffer);
        uint64_t host_user_addr = pci_dma_buffer_get_user_addr (transfer_buffer);
        while (num_bytes > 0) {
            uint32_t transfered_bytes = std::min<uint32_t>(num_bytes, dma_buf_size);
            memcpy ( (void*)host_user_addr, (void*)buffer_addr, transfered_bytes);
            pcie_dma_transfer_turbo (dev, byte_addr, host_phys_addr, transfered_bytes, true);
            num_bytes -= transfered_bytes;
            byte_addr += transfered_bytes;
            buffer_addr += transfered_bytes;
        }
        return;
    }

    record_access("write_block_b", byte_addr, num_bytes, false, true, true, false); // addr, size, turbo, write, block, endline

    auto reg_mapping = (char *)register_address<char>(dev, byte_addr);

    void *dest = reinterpret_cast<char *>(reg_mapping);
    const void *src = reinterpret_cast<const void *>(buffer_addr);
#ifndef DISABLE_ISSUE_3487_FIX
    memcpy_to_device(dest, src, num_bytes);
#else
#ifdef FAST_MEMCPY
    memcpy_to_device(dest, src, num_bytes);
   if ((num_bytes % 32 == 0) && ((intptr_t(dest) & 31) == 0) && ((intptr_t(src) & 31) == 0))

   {
      // Faster memcpy version.. about 8x currently compared to pci_read above
      fastMemcpy(dest, src, num_bytes);
   }
   else
#else
     // ~4x faster than pci_read above, but works for all sizes and alignments
     memcpy(dest, src, num_bytes);
#endif
#endif
    print_buffer (buffer_addr, std::min(g_NUM_BYTES_TO_PRINT, num_bytes), true);
}

void read_checking_enable(bool enable = true) {
    g_READ_CHECKING_ENABLED = enable;
}

// Read/write to the configuration space of the device
// pData is a pointer to a buffer (see memory module)
DWORD read_cfg(TTDevice *dev, DWORD byte_offset, uint64_t pData, DWORD num_bytes) {

    if (pread(get_config_space_fd(dev), reinterpret_cast<void*>(pData), num_bytes, byte_offset) != num_bytes) {
        throw std::runtime_error("Config space read failed for device ");
    }

    return 0;
}

DWORD write_cfg(TTDevice *dev, DWORD byte_offset, uint64_t pData, DWORD num_bytes) {

    if (pwrite(get_config_space_fd(dev), reinterpret_cast<const void*>(pData), num_bytes, byte_offset) != num_bytes) {
        throw std::runtime_error("Config space read failed for device ");
    }

    return 0;
}

DMAbuffer pci_allocate_dma_buffer(TTDevice *dev, uint32_t size) {

    uint32_t page_size = getpagesize();
    uint32_t page_aligned_size = (size + page_size - 1) & ~(page_size - 1);

    DMAbuffer ret_val = allocate_dma_buffer(dev, dev->next_dma_buf++, page_aligned_size);
    LOG1 ("Allocated DMA buffer at 0x%lx 0x%lx size: %u\n", ret_val.pBuf, ret_val.pDma, size);
    return ret_val;
}

void pcie_init_dma_transfer_turbo () {
                                                     // From SHA 8cf7ff1bc7b3886a:
    c_CSM_PCIE_CTRL_DMA_REQUEST_OFFSET = 0x1fef84c0; // chip.AXI.get_path_info("ARC_CSM.ARC_PCIE_DMA_REQUEST")
    c_DMA_TRIGGER_ADDRESS = 0x1ff30074;              // chip.AXI.get_path_info("ARC_RESET.SCRATCH[5]")
    c_ARC_MISC_CNTL_ADDRESS = 0x1ff30100;            // chip.AXI.get_path_info("ARC_RESET.ARC_MISC_CNTL")
}

void set_use_dma(bool msi, uint32_t dma_block_size_read_threshold_bytes, uint32_t dma_block_size_write_threshold_bytes) {
    g_USE_MSI_FOR_DMA = msi;
    g_DMA_BLOCK_SIZE_READ_THRESHOLD_BYTES  = dma_block_size_read_threshold_bytes;
    g_DMA_BLOCK_SIZE_WRITE_THRESHOLD_BYTES = dma_block_size_write_threshold_bytes;
}

void write_regs(TTDevice *dev, uint32_t byte_addr, uint32_t word_len, const void *data) {
    record_access("write_regs", byte_addr, word_len * sizeof(uint32_t), false, true, false, false);

    volatile uint32_t *dest = register_address<std::uint32_t>(dev, byte_addr);
    const uint32_t *src = reinterpret_cast<const uint32_t*>(data);

    while (word_len-- != 0) {
        uint32_t temp;
        memcpy(&temp, src++, sizeof(temp));
        *dest++ = temp;
    }
    LOG2(" REG ");
    print_buffer (reinterpret_cast<uint64_t>(data), std::min(g_NUM_BYTES_TO_PRINT, word_len * 4), true);
}

void write_tlb_reg(TTDevice *dev, uint32_t byte_addr, std::uint64_t value) {
    record_access("write_tlb_reg", byte_addr, sizeof(value), false, true, false, false);

    volatile uint64_t *dest = register_address<std::uint64_t>(dev, byte_addr);
    *dest = value;
    _mm_mfence(); // Otherwise subsequent WC loads move earlier than the above UC store to the TLB register.

    LOG2(" TLB ");
    print_buffer (reinterpret_cast<uint64_t>(&value), sizeof(value), true);
}

void read_regs(TTDevice *dev, uint32_t byte_addr, uint32_t word_len, void *data) {
    record_access("read_regs", byte_addr, word_len * sizeof(uint32_t), false, false, false, false);

    const volatile uint32_t *src = register_address<std::uint32_t>(dev, byte_addr);
    uint32_t *dest = reinterpret_cast<uint32_t*>(data);

    while (word_len-- != 0) {
        uint32_t temp = *src++;
        memcpy(dest++, &temp, sizeof(temp));
    }
    LOG2(" REG ");
    print_buffer (reinterpret_cast<uint64_t>(data), std::min(g_NUM_BYTES_TO_PRINT, word_len * 4), true);
}

void handle_dma_timeout(TTDevice *dev, uint32_t size_bytes, bool write) {
    detect_ffffffff_read(dev);
    throw std::runtime_error(std::string("DMA transfer timeout: ")
                             + std::to_string(size_bytes)
                             + (write ? " byte write." : " byte read."));
}

uint32_t pcie_dma_transfer_turbo (TTDevice *dev, uint32_t chip_addr, uint32_t host_phys_addr, uint32_t size_bytes, bool write) {
    // c_timer t ("");

    // t.now_in ("1. DMA setup");

    if (c_CSM_PCIE_CTRL_DMA_REQUEST_OFFSET == 0) {
        throw std::runtime_error ("pcie_init_dma_transfer_turbo must be called before pcie_dma_transfer_turbo");
    }

    arc_pcie_ctrl_dma_request_t req = {
        .chip_addr           = chip_addr,
        .host_phys_addr      = host_phys_addr,
        .completion_flag_phys_addr = static_cast<uint32_t>(pci_dma_buffer_get_physical_addr(dev->dma_completion_flag_buffer)),
        .size_bytes          = size_bytes,
        .write               = (write ? 1U : 0U),
        .pcie_msi_on_done    = g_USE_MSI_FOR_DMA ? 1U : 0U,
        .pcie_write_on_done  = g_USE_MSI_FOR_DMA ? 0U : 1U,
        .trigger             = 1U,
        .repeat              = 1
    };

    volatile uint32_t *complete_flag = (uint32_t *)pci_dma_buffer_get_user_addr(dev->dma_completion_flag_buffer);
    *complete_flag = 0;

    // Configure the DMA engine
    msi_interrupt_received = false;
    write_regs (dev, c_CSM_PCIE_CTRL_DMA_REQUEST_OFFSET, sizeof(req) / sizeof(uint32_t), &req);

    // Trigger ARC interrupt 0 on core 0
    int arc_misc_cntl_value = 0;

    // NOTE: Ideally, we should read the state of this register before writing to it, but that
    //       casues a lot of delay (reads have huge latencies)
    arc_misc_cntl_value |= (1 << 16); // Cause IRQ0 on core 0
    write_regs (dev, c_ARC_MISC_CNTL_ADDRESS, 1, &arc_misc_cntl_value);

    if (!g_USE_MSI_FOR_DMA) {
        // t.now_in ("2. DMA poll");
        int wait_loops = 0;
        while (true) {
            // The complete flag is set ty by ARC (see src/hardware/soc/tb/arc_fw/lib/pcie_dma.c)
            if (*complete_flag == 0xfaca) break;
            wait_loops++;
        }
        // LOG2 ("Waited %d iterations\n", wait_loops);
    } else {
        // t.now_in ("2. DMA wait for MSI");
        while (msi_interrupt_received == false)
            ;
    }

    return 0; // TODO: status
}

void print_device_info (struct PCIdevice &d) {
    LOG1("PCIEIntfId   0x%x\n", d.id);
    LOG1("VID:DID      0x%x:0x%x\n", d.vendor_id, d.device_id);
    LOG1("SubVID:SubID 0x%x:0x%x\n", d.subsystem_vendor_id, d.subsystem_id);
    LOG1("BSF          %x:%x:%x\n",  d.dwBus, d.dwSlot, d.dwFunction);
    LOG1("BAR          0x%llx  size: %dMB\n",    d.BAR_addr, d.BAR_size_bytes / 1024 / 1024);
}
