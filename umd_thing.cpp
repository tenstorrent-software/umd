
#include "device/api/umd/device/cluster.h" // LOL

int main()
{
    tt::umd::ClusterX280 device(
        "/root/blackhole_x280.yaml",
        std::set<int>{0}, // target_devices
        1, // num_host_mem_ch_per_mmio_device
        false, // skip_driver_allocs
        true,  // clean_system_resources
        false  // perform_harvesting
    );
    tt_device_l1_address_params l1_address_params;
    tt_device_params default_params;

    l1_address_params.tensix_l1_barrier_base = 12;
    l1_address_params.eth_l1_barrier_base = 0x7ffe0;
    l1_address_params.fw_version_addr = 0x210;

    device.set_device_l1_address_params(l1_address_params);
    device.start_device(default_params);
    device.assert_risc_reset();
    device.close_device();
}
