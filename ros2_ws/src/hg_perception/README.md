# hg_perception

Perception for Home Guardian. Today: a driver for the Melexis MLX90640 thermal camera (32 x 24
pixels, 110 deg version) on the Raspberry Pi's I2C bus, and a false-color snapshot tool. Camera and
detection nodes will join this package later.

## thermal_node (C++)

On start it dumps the sensor EEPROM, extracts the calibration with Melexis's official API
(vendored unchanged in [`third_party/`](third_party/README.md)), and sets the refresh rate and
chess mode. A dedicated thread waits for each new subpage, computes the temperatures (emissivity
from the parameters, reflected temperature = ambient - 8 C), applies the API's bad-pixel
correction, and publishes the full image. Slow I2C never blocks ROS callbacks.

In chess mode each subpage updates half of the pixels, so the image is republished on every
subpage (about `refresh_rate_hz` times per second), starting once both subpages have been read.
On an I2C error (sensor missing, read failure) the node logs it, retries every 2 s and recovers
without a restart.

| Topic | Type | Content |
| --- | --- | --- |
| `/thermal/image` | `sensor_msgs/Image` | 32 x 24, `32FC1`, deg C, row 0 at the top, frame `thermal_link` |

| Parameter | Default | Description |
| --- | --- | --- |
| `i2c_device` | `/dev/i2c-1` | I2C bus device |
| `address` | `0x33` (51) | Sensor I2C address |
| `refresh_rate_hz` | `8` | Subpage rate: 0.5, 1, 2, 4, 8 or 16 at 400 kHz |
| `emissivity` | `0.95` | Object emissivity |
| `flip_horizontal` | `false` | Mirror left and right |
| `flip_vertical` | `false` | Mirror top and bottom |
| `frame_id` | `thermal_link` | Header frame |
| `log_stats` | `true` | Once per second: min, max, mean and center temperature, and the publish rate |

Parameters are read once at start, from [`config/thermal.yaml`](config/thermal.yaml):

```bash
ros2 launch hg_perception thermal.launch.py
```

**Orientation:** the image should look like a normal camera's. An object on the sensor's left,
seen from behind the sensor, should appear on the left of the image. Check it with a snapshot
and set `flip_horizontal` / `flip_vertical` in `config/thermal.yaml` if needed.

## thermal_snapshot (Python)

Takes one `/thermal/image` message and saves a false-color PNG, scaled to 320 x 240, as
`~/thermal_snapshots/thermal_<date>_<time>_min<..>C_max<..>C.png`. It prints the path, min and
max.

```bash
ros2 run hg_perception thermal_snapshot
```

The coloring lives in `hg_perception.false_color` (standard library only) for reuse, for
example by event alerts.

## Requirements

Only packages that come with ROS 2 Jazzy (`ros-base`, `ros-dev-tools`). I2C must be enabled and
the user must be in the `i2c` group (see [docs/setup/raspberry-pi.md](../../../docs/setup/raspberry-pi.md)).

## License

Apache-2.0. The vendored Melexis MLX90640 API is also Apache-2.0; its license file is in
`third_party/mlx90640-library/LICENSE`.
