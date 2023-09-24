#include "luwen_impl.h"

#include <boost/interprocess/sync/scoped_lock.hpp>
#include <boost/interprocess/sync/named_mutex.hpp>

#include "pci_comms.h"
#include "common/logger.hpp"

luwen::DeviceInfo device_info_func(void *user_data) {
    auto *device = (DeviceRef *)user_data;

    auto pci_device = device->get_pci_device(device->chip_id);

    return luwen::DeviceInfo {
        pci_device->id,

        0,
        pci_device->dwBus,
        pci_device->dwSlot,
        pci_device->dwFunction,

        pci_device->vendor_id,
        pci_device->device_id,
        pci_device->BAR_size_bytes,
    };
}

void bar_read(uint32_t addr, uint8_t *data, uint32_t len, void *user_data) {
    auto *device = (DeviceRef *)user_data;

    auto pci_device = device->get_pci_device(device->chip_id)->hdev;

    read_block(pci_device, addr, len, (uint64_t)data, device->dma_buf_size());
}

void bar_write(uint32_t addr, const uint8_t *data, uint32_t len, void *user_data) {
    auto *device = (DeviceRef *)user_data;

    auto pci_device = device->get_pci_device(device->chip_id)->hdev;

    write_block(pci_device, addr, len, (uint64_t)data, device->dma_buf_size());
}

void noc_read(uint8_t noc_id,
                uint32_t x,
                uint32_t y,
                uint64_t addr,
                uint8_t *data,
                uint64_t len,
                void *user_data) {
    auto *device = (DeviceRef *)user_data;

    device->read_device_memory((uint32_t *)data, len, tt_cxy_pair{device->chip_id, tt_xy_pair{x, y}}, addr, "SMALL_READ_WRITE_TLB");
}

void noc_write(uint8_t noc_id,
                uint32_t x,
                uint32_t y,
                uint64_t addr,
                const uint8_t *data,
                uint64_t len,
                void *user_data) {
    auto *device = (DeviceRef *)user_data;

    device->write_device_memory((uint32_t *)data, len, tt_cxy_pair{device->chip_id, tt_xy_pair{x, y}}, addr, "SMALL_READ_WRITE_TLB");
}

void noc_broadcast(uint8_t noc_id,
                    uint64_t addr,
                    const uint8_t *data,
                    uint64_t len,
                    void *user_data) {
    auto *device = (DeviceRef *)user_data;

    device->broadcast_to_device_memory((uint32_t *)data, len, device->chip_id, addr, "SMALL_READ_WRITE_TLB");
}

void eth_read(luwen::EthAddr eth_addr,
                uint8_t noc_id,
                uint32_t x,
                uint32_t y,
                uint64_t addr,
                uint8_t *data,
                uint64_t len,
                void *user_data) {
    auto *device = (DeviceRef *)user_data;

    device->read_remote_device_memory(
        (uint32_t *)data, len,
        eth_coord_t {eth_addr.shelf_x, eth_addr.shelf_y, eth_addr.rack_x, eth_addr.rack_y},
        tt_xy_pair{x, y}, addr
    );
}

void eth_write(luwen::EthAddr eth_addr,
                uint8_t noc_id,
                uint32_t x,
                uint32_t y,
                uint64_t addr,
                const uint8_t *data,
                uint64_t len,
                void *user_data) {
    auto *device = (DeviceRef *)user_data;

    device->write_remote_device_memory(
        (uint32_t *)data, len,
        eth_coord_t {eth_addr.shelf_x, eth_addr.shelf_y, eth_addr.rack_x, eth_addr.rack_y},
        tt_xy_pair{x, y}, addr
    );
}

void eth_broadcast(luwen::EthAddr eth_addr,
                    uint8_t noc_id,
                    uint64_t addr,
                    const uint8_t *data,
                    uint64_t len,
                    void *user_data) {
    auto *device = (DeviceRef *)user_data;

    throw std::runtime_error("NO SUPPORT FOR ETH BROADCAST");
}

LuwenChip::LuwenChip(const tt::ARCH &arch, DeviceRef *device) {
    luwen::Arch larch;
    switch (arch) {
        case tt::ARCH::GRAYSKULL: {
            larch = luwen::Arch::GRAYSKULL;
            break;
        }
        case tt::ARCH::WORMHOLE:
        case tt::ARCH::WORMHOLE_B0: {
            larch = luwen::Arch::WORMHOLE;
            break;
        }
        default:
            throw std::runtime_error("Unsupported ARCH " + get_arch_str(arch));
    }

    auto ldevice = luwen::LuwenGlue {
        (void *)device,
        &device_info_func,
        &bar_read,
        &bar_write,
        &noc_read,
        &noc_write,
        &noc_broadcast,
        &eth_read,
        &eth_write,
        &eth_broadcast
    };

    this->chip = luwen::luwen_open(larch, ldevice);
}

LuwenChip::LuwenChip(LuwenChip &local, eth_coord_t eth_addr) {
    auto addr = luwen::EthAddr {
        std::get<0>(eth_addr),
        std::get<1>(eth_addr),
        std::get<2>(eth_addr),
        std::get<3>(eth_addr)
    };

    this->chip = luwen::luwen_open_remote(local.chip, addr);
}

int LuwenChip::arc_msg(uint32_t msg, bool wait_for_done, uint32_t arg0, uint32_t arg1, int32_t timeout, uint32_t *result) {
    luwen::CResult cresult = luwen::chip_arc_msg(this->chip, msg, wait_for_done, arg0, arg1, timeout, result);
    if (cresult.tag == luwen::CResultTag::Ok) {
        return cresult.ok;
    } else {
        throw std::runtime_error(cresult.err);
    }
}

void LuwenChip::init() {
    luwen::chip_init(this->chip);
}

struct PCIdevice *DeviceRef::get_pci_device(chip_id_t chip_id) {
    return this->device->get_pci_device(chip_id);
}

void DeviceRef::write_device_memory(
        uint32_t *data, uint64_t len, tt_cxy_pair cxy,
        uint64_t addr, const char *tlb_name
) {
    this->device->write_device_memory(data, len, cxy, addr, tlb_name);
}

void DeviceRef::read_device_memory(
        uint32_t *data, uint64_t len, tt_cxy_pair cxy,
        uint64_t addr, const char *tlb_name
) {
    this->device->read_device_memory(data, cxy, addr, len, tlb_name);
}

void DeviceRef::broadcast_to_device_memory(
        uint32_t *data, uint64_t len, chip_id_t chip_id,
        uint64_t addr, const char *fallback_tlb
) {
    struct PCIdevice* pci_device = device->get_pci_device(chip_id);
    TTDevice *dev = pci_device->hdev;

    std::uint32_t size_in_bytes = len * sizeof(std::uint32_t);
    uint64_t buffer_addr = (uint64_t)data;

    const auto tlb_index = device->dynamic_tlb_config.at(fallback_tlb);
    const boost::interprocess::scoped_lock<boost::interprocess::named_mutex> lock(*device->get_mutex(fallback_tlb, pci_device->id));

    auto address = addr;
    while(size_in_bytes > 0) {
        auto [mapped_address, tlb_size] = set_dynamic_tlb_broadcast(pci_device, tlb_index, address, device->harvested_coord_translation, true);
        uint32_t transfer_size = std::min(size_in_bytes, tlb_size);
        write_block(dev, mapped_address, transfer_size, buffer_addr, device->m_dma_buf_size);

        size_in_bytes -= transfer_size;
        address += transfer_size;
        buffer_addr += transfer_size;
    }
}

void DeviceRef::write_remote_device_memory(
    uint32_t *data, uint64_t len, eth_coord_t eth_addr, tt_xy_pair core, uint64_t addr
) {
    tt_device_logger::log_assert((device->get_soc_descriptor(this->chip_id).ethernet_cores).size() > 0 && device->get_number_of_chips_in_cluster() > 1, "Cannot issue ethernet writes to a single chip cluster!");
    boost::interprocess::named_mutex named_mtx(boost::interprocess::open_or_create, "non_mmio_mutex");
    named_mtx.lock();
    device->write_to_non_mmio_device(data, len, this->chip_id, eth_addr, core, addr);
    named_mtx.unlock();
}

void DeviceRef::read_remote_device_memory(
        uint32_t *data, uint64_t len, eth_coord_t eth_addr, tt_xy_pair core, uint64_t addr
) {
    tt_device_logger::log_assert((device->get_soc_descriptor(this->chip_id).ethernet_cores).size() > 0 && device->get_number_of_chips_in_cluster() > 1, "Cannot issue ethernet reads from a single chip cluster!");
    boost::interprocess::named_mutex named_mtx(boost::interprocess::open_or_create, "non_mmio_mutex");
    named_mtx.lock();
    device->read_from_non_mmio_device(data, this->chip_id, eth_addr, core, addr, len);
    named_mtx.unlock();
}

uint32_t DeviceRef::dma_buf_size() {
    return device->m_dma_buf_size;
}
