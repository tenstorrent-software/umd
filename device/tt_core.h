/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "pcie/pci_device.hpp"
#include "tt_xy_pair.h"
#include "tt_tlb_manager.h"

/*

*** Things to consider for future design ***

- We are going to have LocalChip and RemoteChip classes that inherit from TTChip.
Our constructor is going to have those classes as parameters instead of tt_device. 

*** Usage ***

Idea is that writing to core (remote or local) is transparent for the user

Usage could look something like

TTChip* local_chip = new LocalChip();
TTCore* local_core = local_chip.get_core(0, 0);
local_core->reset();
// this is going to take TLB of local chip for core 0,0 and write data to it
local_core->write(offset, data, size);

TTChip* remote_chip = new RemoteChip();
TTCore* remote_core = remote_chip.get_core(0, 0);
remote_core->reset();
// this is going to take TLB from the local chip tied to remote chip for the ETH core and write commands
// to it such that data is written to core 0,0 of the remote chip
remote_core->write(offset, data, size);

*/

class tt_SiliconDevice;

namespace tt {

class tt_Core {

public:
    tt_Core(tt_xy_pair core_pair_, tt::TLBManager* tlb_manager_) : core_pair(core_pair_), tlb_manager(tlb_manager) {}

    virtual void write(uint32_t address, void* src_addr, size_t size) = 0;

    tt_xy_pair get_core_pair() {
        return core_pair;
    }
protected:
    tt_xy_pair core_pair;
    // We can have either TTChip, or PCIDevice, or anything else, TLB manager is here just to show the idea.
    // It is supposed to encapsulate the logic of getting TLB Writers.
    tt::TLBManager* tlb_manager = nullptr;
    tt:Writer tlb_writer;
}

class tt_LocalCore : public tt_Core {
public:
    tt_LocalCore(tt_xy_pair core_pair_, tt::TLBManager* tlb_manager_) : TTCore(core_pair_, tlb_manager_) {
        tlb_writer = tlb_manager->get_static_tlb_writer(core_pair);
    }

    void write(uint32_t address, void* src_addr, size_t size) {
        tlb_writer.write(address, src_addr, size);
    }

    ~tt_LocalCore() {
        tlb_manager->release_tlb_writer(tlb_writer);
    }
}

class RemoteCore : public tt_Core {
    tt_RemoteCore(tt_xy_pair core_pair_, tt::TLBManager* tlb_manager_) : tt_Core(core_pair_, tlb_manager_) {}

    void write(uint32_t address, void* src_addr, size_t size) {
        // dummy eth core
        tt_xy_pair eth_core = tt_xy_pair(0, 0);
        
        // this is going to be taken from local chip in the future
        // TODO: currently there is a mutex on read/writing to the non mmio device,
        // should this function for getting the writer provide the synchronization?
        tt::Writer tlb_writer = tlb_manager->get_tlb_writer(eth_core);

        // prepare eth commands similar to write_to_non_mmio_device function in tt_silicon_driver.cpp
        // ...
        void* cmd = nullptr;
        size_t cmd_size = 0;
            
        // write to the eth core of the remote chip
        tlb_writer.write(address, cmd, cmd_size);

        // release the writer
        tlb_manager->release_tlb_writer(tlb_writer);
    }
}

}