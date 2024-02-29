#include <iostream>

#include "device/device_collection.h"
#include "device/kmd.h"

// TODO: Remove this app from code :)

int main() {
    // Testing kmd class
    std::cout << "Testing kmd class" << std::endl;
    auto chip_ids = tt::umd::kmd::scan();

    for (auto chip_id : chip_ids) {
        std::cout << "Chip id: " << chip_id << std::endl;
        try {
            auto kmd = tt::umd::kmd::open(chip_id);
            std::cout << "  Open succeeded" << std::endl;
            std::cout << "  Architecture: " << kmd->get_architecture() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "  Failed to open kmd: " << e.what() << std::endl;
        }
    }

    // Testing device_collection class
    std::cout << std::endl << "Testing device_collection class" << std::endl;
    auto host_devices = tt::umd::device_collection::host_devices();

    for (const auto& device : host_devices.get_devices()) {
        if (device) {
            std::cout << "Logical device id: " << device->get_logical_device_id() << std::endl;
            std::cout << "  Architecture: " << device->get_architecture() << std::endl;
        }
    }

    return 0;
}
