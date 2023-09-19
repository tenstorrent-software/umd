#pragma once

#include <cstdarg>
#include <cstdint>
#include <cstdlib>
#include <ostream>
#include <new>

namespace luwen {

enum class Arch : uint8_t {
  GRAYSKULL,
  WORMHOLE,
};

enum class CResultTag : uint8_t {
  Ok,
  Err,
};

struct DeviceInfo {
  uint32_t interface_id;
  uint16_t domain;
  uint16_t bus;
  uint16_t device;
  uint16_t function;
};

struct EthAddr {
  uint8_t shelf_x;
  uint8_t shelf_y;
  uint8_t rack_x;
  uint8_t rack_y;
};

struct LuwenGlue {
  void *user_data;
  DeviceInfo (*device_info)(void *user_data);
  /// Impls for bar reads and writes, the lowest level of communication
  /// used by local chips to talk to ARC.
  void (*axi_read)(uint32_t addr, uint8_t *data, uint32_t len, void *user_data);
  void (*axi_write)(uint32_t addr, const uint8_t *data, uint32_t len, void *user_data);
  /// Impls for noc reads and writes
  void (*noc_read)(uint8_t noc_id,
                   uint32_t x,
                   uint32_t y,
                   uint64_t addr,
                   uint8_t *data,
                   uint64_t len,
                   void *user_data);
  void (*noc_write)(uint8_t noc_id,
                    uint32_t x,
                    uint32_t y,
                    uint64_t addr,
                    const uint8_t *data,
                    uint64_t len,
                    void *user_data);
  void (*noc_broadcast)(uint8_t noc_id,
                        uint64_t addr,
                        const uint8_t *data,
                        uint64_t len,
                        void *user_data);
  /// Impls for eth reads and writes could be implemented with noc operations but
  /// requires exclusive access to the erisc being used. Managing this is left to the implementor.
  void (*eth_read)(EthAddr eth_addr,
                   uint8_t noc_id,
                   uint32_t x,
                   uint32_t y,
                   uint64_t addr,
                   uint8_t *data,
                   uint64_t len,
                   void *user_data);
  void (*eth_write)(EthAddr eth_addr,
                    uint8_t noc_id,
                    uint32_t x,
                    uint32_t y,
                    uint64_t addr,
                    const uint8_t *data,
                    uint64_t len,
                    void *user_data);
  void (*eth_broadcast)(EthAddr eth_addr,
                        uint8_t noc_id,
                        uint64_t addr,
                        const uint8_t *data,
                        uint64_t len,
                        void *user_data);
};

struct CResult {
  CResultTag tag;
  uint32_t ok;
  const char *err;
};

struct Telemetry {
  uint64_t board_id;
};

extern "C" {

Chip *luwen_open(Arch arch, LuwenGlue glue);

Chip *luwen_open_remote(Chip *local_chip, EthAddr addr);

void luwen_close(Chip *chip);

void chip_init(const Chip *chip);

CResult chip_arc_msg(const Chip *chip,
                     uint32_t msg,
                     bool wait_for_done,
                     uint16_t arg0,
                     uint16_t arg1,
                     int32_t timeout,
                     uint32_t *return_3);

Telemetry chip_telemetry(const Chip *chip);

} // extern "C"

} // namespace luwen
