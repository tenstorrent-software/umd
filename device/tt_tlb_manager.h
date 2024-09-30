// SPDX-FileCopyrightText: (c) 2024 Tenstorrent Inc.
//
// SPDX-License-Identifier: Apache-2.0


/*

Idea of the TLB manager is to provide a way to manage TLBs of the MMIO chip.

Since we are discussing multiple designs at the moment, I think it would be useful to encapsulate logic
for managing TLB windows inside a class, define the API and desired behaviour of the API  of the manager.

When we have everything defined and agreed upon, we can implement the desired logic in the TLB manager.

For now, it would just manage whole BAR0 and BAR4 space. When we switch to KMD being the main component for managing
resources, we can change TLB manager logic to make appropriate calls to KMD, but the interface should remain the same, and
other classes should not be affected by the change. 

I am not pushing for the TLB manager to be separate class, just suggesting it as a way to encapsulate logic for managing TLBs. If we
want to move the implementation either to future PCIDevice or TTChip class, it would make sense as well.

tt_device should be changed to TTChip or PCIDevice, depending on the final design.

*/

#include "pcie/pci_device.hpp"
#include "tt_xy_pair.h"
#include "tt_io.hpp"

namespace tt {

class TLBManager {

public:

    TLBManager() {
        // TODO: do we need arch specific TLB count?
        static constexpr TLB_COUNT = 202;
        tlb_counter.resize(TLB_COUNT, 0);
    }

    tt::Writer get_tlb_writer(tt_xy_pair core_pair) {
        auto tlb_index = map_core_to_tlb_index(core_pair);
        auto tlb_data = pci_device->get_architecture_implementation()->describe_tlb(tlb_index);

        if (!tlb_data.has_value()) {
            throw std::runtime_error("No TLB mapped to core " + target.str());
        }

        auto [tlb_offset, tlb_size] = tlb_data.value();
        auto *base = reinterpret_cast<uint8_t *>(pci_device->bar0_wc);
        
        // KMD: counting how many times the TLB is used to return the resource to KMD if it is not used anymore.
        tlb_counter[index]++;
        // end KMD

        return tt::Writer(base + tlb_offset, tlb_size);
    }

    void release_tlb_writer(tt::Writer writer) {
        // KMD: counting how many times the TLB is used to return the resource to KMD if it is not used anymore.
        tlb_counter[writer.index]--;

        if (tlb_counter[writer.index] == 0) {
            // KMD call to return the TLB resource
        }
        // end KMD
    }

private:
    PCIDevice* pci_device;
    
    std::vector<uint32_t> tlb_counter;

    // dummy implementation
    tt_xy_pair map_core_to_tlb_index(tt_xy_pair core_pair) {
        return 0;
    }
}

}