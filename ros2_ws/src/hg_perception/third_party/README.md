# Third-party code

## mlx90640-library

Melexis's official MLX90640 API, vendored **unchanged** (calibration extraction, temperature
calculation, bad-pixel correction).

| | |
| --- | --- |
| Source | https://github.com/melexis/mlx90640-library |
| Commit | `f6be7ca1d4a55146b705f3d347f84b773b29cc86` (2023-02-07) |
| Files | `functions/MLX90640_API.c`, `headers/MLX90640_API.h`, `headers/MLX90640_I2C_Driver.h`, `LICENSE` |
| License | Apache License 2.0 (`mlx90640-library/LICENSE`), Copyright Melexis N.V. |

The I2C functions declared in `MLX90640_I2C_Driver.h` are implemented by this package in
`src/mlx90640_linux_i2c.cpp` (Linux `I2C_RDWR` ioctl on `/dev/i2c-*`). To update, copy the same
files from a newer upstream commit and record it here. Do not edit the vendored files.
