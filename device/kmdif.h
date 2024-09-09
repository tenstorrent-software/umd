/*
 * SPDX-FileCopyrightText: (c) 2023 Tenstorrent Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include <cstdint>

typedef std::uint32_t DWORD;

const uint32_t MAX_DMA_BYTES = 4*1024*1024;

// DMA
struct DMAbuffer {
	void *pBuf = NULL;
	std::uint64_t pDma = 0;
	std::uint64_t size;
};

struct TTDevice;

namespace tt::umd {

enum class chip_id : int;

}  // namespace tt::umd

struct PCIdevice  {
	tt::umd::chip_id id{0};
	TTDevice *hdev = nullptr;
    tt::umd::chip_id logical_id;
    std::uint16_t vendor_id;
    std::uint16_t device_id;
    std::uint16_t subsystem_vendor_id;
    std::uint16_t subsystem_id;
    std::uint16_t revision_id;

    // PCI bus identifiers
    DWORD dwDomain;
	DWORD dwBus;
    DWORD dwSlot;
    DWORD dwFunction;

	uint64_t BAR_addr;
	DWORD BAR_size_bytes;
};