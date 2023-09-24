#pragma once

#include <cstdint>
#include <optional>

namespace luwen {
struct Chip;
struct DeviceInfo;
}

#include <luwen.h>

#include "tt_arch_types.h"
#include "tt_soc_descriptor.h"
#include "tt_cluster_descriptor_types.h"

struct DeviceRef;

struct LuwenChip {
public:
    LuwenChip(const tt::ARCH &arch, DeviceRef *device);
    LuwenChip(LuwenChip &local, eth_coord_t eth_addr);

    void init();

    int arc_msg(uint32_t msg, bool wait_for_done, uint32_t arg0, uint32_t arg1, int32_t timeout, uint32_t *result);
    luwen::Telemetry get_telemetry();

private:
    luwen::Chip *chip;
};

class tt_SiliconDevice;

#include "kmdif.h"
#include "tt_device.h"

struct DeviceRef {
public:
    tt_SiliconDevice *device;
    chip_id_t chip_id;

    struct PCIdevice *get_pci_device(chip_id_t chip_id);
    void write_device_memory(
        uint32_t *data, uint64_t len, tt_cxy_pair cxy,
        uint64_t addr, const char *tlb_name
    );
    void read_device_memory(
        uint32_t *data, uint64_t len, tt_cxy_pair cxy,
        uint64_t addr, const char *tlb_name
    );
    void broadcast_to_device_memory(
        uint32_t *data, uint64_t len, chip_id_t chip_id,
        uint64_t addr, const char *fallback_tlb
    );
    void write_remote_device_memory(
        uint32_t *data, uint64_t len, eth_coord_t eth_addr, tt_xy_pair core, uint64_t addr
    );
    void read_remote_device_memory(
        uint32_t *data, uint64_t len, eth_coord_t eth_addr, tt_xy_pair core, uint64_t addr
    );

    uint32_t dma_buf_size();
};
