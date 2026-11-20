// SPDX-License-Identifier: Apache-2.0
//
// Linux I2C back end for the Melexis MLX90640 API.
//
// src/mlx90640_linux_i2c.cpp implements the functions declared in the vendored
// MLX90640_I2C_Driver.h with the I2C_RDWR ioctl on /dev/i2c-*. The Melexis API calls those
// functions without a device handle, so the open device is process-wide: one sensor per process,
// used from one thread at a time.

#ifndef HG_PERCEPTION_MLX90640_LINUX_I2C_HPP_
#define HG_PERCEPTION_MLX90640_LINUX_I2C_HPP_

#include <string>

namespace hg::mlx90640_i2c {

// Opens the I2C bus device (for example /dev/i2c-1), closing any previously open one. Returns
// false on failure; last_errno() then holds the reason.
bool open_device(const std::string& path);

// Closes the device. Safe to call when nothing is open.
void close_device();

// errno of the last failed open or I2C transfer.
int last_errno();

}  // namespace hg::mlx90640_i2c

#endif  // HG_PERCEPTION_MLX90640_LINUX_I2C_HPP_
