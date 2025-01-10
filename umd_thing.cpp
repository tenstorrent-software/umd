
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

    tt_device_params default_params;
    device.start_device(default_params);
    device.assert_risc_reset();
    device.close_device();
}
