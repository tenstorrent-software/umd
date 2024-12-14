/* SPDX-FileCopyrightText: © 2024 Tenstorrent Inc.
 * SPDX-License-Identifier: GPL-2.0-only
 *
 * Tenstorrent L2CPU NOC Window Management Driver Interface
 *
 * The L2CPU is a hardware block in the Tenstorrent Blackhole chip containing
 * four RISC-V cores and a Network-on-Chip (NOC) interface.  This driver runs on
 * the RISC-V cores and manages access to the rest of the chip through
 * configurable NOC windows.
 *
 * This header defines the userspace interface for allocating, configuring, and
 * deallocating NOC windows.  Hardware provides two window sizes:
 *
 * 2M (2 MiB; 0x200000 bytes) - 224 windows
 * 128G (128 GiB; 0x2000000000 bytes) - 32 windows
 */

#ifndef _L2CPU_NOC_IOCTL_H
#define _L2CPU_NOC_IOCTL_H

#include <linux/ioctl.h>
#include <linux/types.h>

/**
 * struct noc_window_handle - Handle for a NOC window
 * @window_id: Driver's handle for this window
 * @mmap_offset: Offset to pass to mmap() to map this window
 * @mmap_size: Size available at this offset
 */
struct noc_window_handle {
	__u32 window_id;
	__u64 mmap_offset;
	__u64 mmap_size;
};

/**
 * struct noc_window_config - Configuration parameters for a NOC window
 * @window_id: Window ID from noc_window_handle
 * @addr: Base address for the window
 * @x_end: Target NOC X coordinate (unicast) or end of range (multicast)
 * @y_end: Target NOC Y coordinate (unicast) or end of range (multicast)
 * @x_start: Start of multicast X range (set to 0 for unicast)
 * @y_start: Start of multicast Y range (set to 0 for unicast)
 * @multicast_en: Enable multicast mode to route to x/y coordinate range (1 bit)
 * @strict_order: Enforce strict ordering (1 bit)
 * @posted: Enable posted writes (1 bit)
 * @linked: Enable linked mode (1 bit)
 * @static_en: Enable static routing (1 bit)
 * @stream_header: Enable stream headers (1 bit)
 * @noc_selector: NOC path selector (1 bit)
 * @static_vc: Static virtual channel (3 bits)
 */
struct noc_window_config {
	__u32 window_id;
	__u64 addr;
	__u32 x_end;
	__u32 y_end;
	__u32 x_start;
	__u32 y_start;
	__u8 multicast_en;
	__u8 strict_order;
	__u8 posted;
	__u8 linked;
	__u8 static_en;
	__u8 stream_header;
	__u8 noc_selector;
	__u8 static_vc;
};

/* IOCTL definitions */
#define L2CPU_IOC_MAGIC 'N'

#define L2CPU_IOCTL_ALLOC_2M _IOR(L2CPU_IOC_MAGIC, 1, struct noc_window_handle)
#define L2CPU_IOCTL_ALLOC_128G _IOR(L2CPU_IOC_MAGIC, 2, struct noc_window_handle)

#define L2CPU_IOCTL_CONFIG_2M _IOW(L2CPU_IOC_MAGIC, 3, struct noc_window_config)
#define L2CPU_IOCTL_CONFIG_128G _IOW(L2CPU_IOC_MAGIC, 4, struct noc_window_config)

#define L2CPU_IOCTL_DEALLOC_2M _IOW(L2CPU_IOC_MAGIC, 5, struct noc_window_handle)
#define L2CPU_IOCTL_DEALLOC_128G _IOW(L2CPU_IOC_MAGIC, 6, struct noc_window_handle)

#endif /* _L2CPU_NOC_IOCTL_H */