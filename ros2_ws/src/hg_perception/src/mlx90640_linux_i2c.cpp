// SPDX-License-Identifier: Apache-2.0
//
// Linux I2C driver for the Melexis MLX90640 API (MLX90640_I2C_Driver.h), using the I2C_RDWR
// ioctl on /dev/i2c-*.
//
// The MLX90640 uses 16-bit register addresses and 16-bit big-endian data words. A read is one
// combined transaction: write the start address, repeated start, read 2 bytes per word. The bus
// speed comes from /boot/firmware/config.txt, so MLX90640_I2CFreqSet() does nothing.

#include "hg_perception/mlx90640_linux_i2c.hpp"

#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cstdint>

extern "C" {
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
}

namespace {

int g_fd = -1;
int g_last_errno = 0;

// Long reads (EEPROM dump: 832 words, frame: 768 words) are split into transfers of at most this
// many words to stay well inside I2C adapter transfer limits.
constexpr uint16_t kMaxWordsPerTransfer = 128;

bool transfer(struct i2c_msg* msgs, uint32_t count) {
  if (g_fd < 0) {
    g_last_errno = EBADF;
    return false;
  }
  struct i2c_rdwr_ioctl_data request;
  request.msgs = msgs;
  request.nmsgs = count;
  if (ioctl(g_fd, I2C_RDWR, &request) < 0) {
    g_last_errno = errno;
    return false;
  }
  return true;
}

}  // namespace

namespace hg::mlx90640_i2c {

bool open_device(const std::string& path) {
  close_device();
  g_fd = ::open(path.c_str(), O_RDWR | O_CLOEXEC);
  if (g_fd < 0) {
    g_last_errno = errno;
    return false;
  }
  return true;
}

void close_device() {
  if (g_fd >= 0) ::close(g_fd);
  g_fd = -1;
}

int last_errno() { return g_last_errno; }

}  // namespace hg::mlx90640_i2c

extern "C" {

void MLX90640_I2CInit(void) {
  // Nothing to do: hg::mlx90640_i2c::open_device() opens the bus.
}

void MLX90640_I2CFreqSet(int /*freq*/) {
  // Nothing to do: the bus speed is set in /boot/firmware/config.txt.
}

int MLX90640_I2CGeneralReset(void) {
  // I2C general call (address 0x00) with the reset command 0x06.
  uint8_t command = 0x06;
  struct i2c_msg msg;
  msg.addr = 0x00;
  msg.flags = 0;
  msg.len = 1;
  msg.buf = &command;
  return transfer(&msg, 1) ? MLX90640_NO_ERROR : -MLX90640_I2C_NACK_ERROR;
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress, uint16_t nMemAddressRead,
                     uint16_t* data) {
  uint8_t buffer[2 * kMaxWordsPerTransfer];
  uint16_t done = 0;
  while (done < nMemAddressRead) {
    const uint16_t words = std::min<uint16_t>(nMemAddressRead - done, kMaxWordsPerTransfer);
    const uint16_t address = static_cast<uint16_t>(startAddress + done);  // Word addresses.
    uint8_t address_bytes[2] = {static_cast<uint8_t>(address >> 8),
                                static_cast<uint8_t>(address & 0xFF)};

    struct i2c_msg msgs[2];
    msgs[0].addr = slaveAddr;
    msgs[0].flags = 0;
    msgs[0].len = 2;
    msgs[0].buf = address_bytes;
    msgs[1].addr = slaveAddr;
    msgs[1].flags = I2C_M_RD;
    msgs[1].len = static_cast<uint16_t>(2 * words);
    msgs[1].buf = buffer;
    if (!transfer(msgs, 2)) return -MLX90640_I2C_NACK_ERROR;

    for (uint16_t i = 0; i < words; ++i) {
      data[done + i] = static_cast<uint16_t>((buffer[2 * i] << 8) | buffer[2 * i + 1]);
    }
    done = static_cast<uint16_t>(done + words);
  }
  return MLX90640_NO_ERROR;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data) {
  uint8_t bytes[4] = {static_cast<uint8_t>(writeAddress >> 8),
                      static_cast<uint8_t>(writeAddress & 0xFF), static_cast<uint8_t>(data >> 8),
                      static_cast<uint8_t>(data & 0xFF)};
  struct i2c_msg msg;
  msg.addr = slaveAddr;
  msg.flags = 0;
  msg.len = 4;
  msg.buf = bytes;
  if (!transfer(&msg, 1)) return -MLX90640_I2C_NACK_ERROR;

  // Read back, as the Melexis reference drivers do. The API ignores a mismatch where one is
  // expected (the status register changes after a write).
  uint16_t readback = 0;
  if (MLX90640_I2CRead(slaveAddr, writeAddress, 1, &readback) != MLX90640_NO_ERROR) {
    return -MLX90640_I2C_NACK_ERROR;
  }
  return readback == data ? MLX90640_NO_ERROR : -MLX90640_I2C_WRITE_ERROR;
}

}  // extern "C"
