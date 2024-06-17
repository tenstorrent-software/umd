#include <iostream>
#include <memory>
#include <random>
#include <set>

#include "device/tt_cluster_descriptor_types.h"
#include "device/tt_device.h"
#include "device/driver_atomics.h"
#include "device/blackhole_implementation.h"

std::unique_ptr<tt_SiliconDevice> make_device()
{
    std::set<chip_id_t> target_devices = {0};
    uint32_t num_host_mem_ch_per_mmio_device = 2;
    std::unordered_map<std::string, std::int32_t> dynamic_tlb_config;
    tt_device_params default_params;

    dynamic_tlb_config["REG_TLB"] = tt::umd::blackhole::REG_TLB;
    auto device = std::make_unique<tt_SiliconDevice>("../tests/soc_descs/blackhole_140_arch_no_eth.yaml",
                                                     "",
                                                     target_devices,
                                                     num_host_mem_ch_per_mmio_device,
                                                     dynamic_tlb_config,
                                                     false, // skip_driver_allocs
                                                     true,  // clean_system_resources
                                                     false   // perform_harvesting
                                                     );
    return device;
}

void fill_with_random_bytes(uint8_t* data, size_t n)
{
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint8_t> dis(0, 255);

    std::generate(data, data + n, [&]() { return dis(gen); });
}

void do_sysmem_test(tt_SiliconDevice& device, size_t num_bytes)
{
    std::vector<uint8_t> buffer(num_bytes, 0x0);
    uint8_t* sysmem = (uint8_t*)device.host_dma_address(0, 0, 0);
    uint64_t pcie_base = device.get_pcie_base_addr_from_device(0);
    
    fill_with_random_bytes(sysmem, num_bytes);

    // Depending on your board... you might need (11, 0) instead of (2, 0).
    const size_t PCIE_X = 2;
    const size_t PCIE_Y = 0;

    // Read from sysmem into buffer using the hardware.
    device.read_from_device(buffer.data(),
                            tt_cxy_pair(0, PCIE_X, PCIE_Y),
                            pcie_base,
                            buffer.size(),
                            "REG_TLB"
    );

    if (std::memcmp(sysmem, buffer.data(), num_bytes) != 0) {
        std::cout << "Failed to read sysmem" << std::endl;
        std::cout << "PCIe base: 0x" << std::hex << pcie_base << std::dec << std::endl;
        std::cout << "sysmem: 0x" << std::hex << (void*)sysmem << std::dec << std::endl;
        std::terminate();
    }

    // Write random data from buffer to sysmem using the hardware.
    fill_with_random_bytes(buffer.data(), buffer.size());
    device.write_to_device(buffer.data(),
                           buffer.size(),
                           tt_cxy_pair(0, PCIE_X, PCIE_Y),
                           pcie_base,
                           "LARGE_WRITE_TLB"
    );
    
    uint32_t throwaway;
    device.read_from_device(&throwaway,
                            tt_cxy_pair(0, PCIE_X, PCIE_Y),
                            pcie_base,
                            sizeof(throwaway),
                            "LARGE_READ_TLB"
    );

    if (std::memcmp(sysmem, buffer.data(), num_bytes) != 0) {
        std::cout << "Failed to write sysmem" << std::endl;
        std::terminate();
    }
    std::cout << "Sysmem test passed (" << num_bytes << " bytes)" << std::endl;
}

int main()
{
    auto device = make_device();
    auto two_megs = 1 << 21;
    do_sysmem_test(*device, two_megs);
}
