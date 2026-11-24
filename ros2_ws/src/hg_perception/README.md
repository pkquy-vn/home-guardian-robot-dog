# hg_perception

Perception for Home Guardian. Today: a driver for the Melexis MLX90640 thermal camera (32 x 24
pixels, 110 deg version) on the Raspberry Pi's I2C bus, simple person and fire detection on its
image, and a false-color snapshot tool. A camera node will join this package later.

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

## thermal_detector (Python)

Simple person and fire detection on every `/thermal/image` frame, standard library only (rules in
[`hg_perception/thermal_detection.py`](hg_perception/thermal_detection.py)). Room temperature is
the median of all 768 pixels.

- **Person** (works in darkness): pixels at least `person_delta_c` above room and at most
  `person_max_c` are grouped into touching clusters (8-neighbour). Clusters at most
  `merge_gap_cols` empty columns apart are merged, so a close person's two legs become one. A
  cluster with at least `person_min_pixels` pixels that is at least `person_min_aspect` times
  taller than wide is a candidate. This is tuned for standing people seen by a camera 19 cm above
  the floor; pets and wide objects such as TVs are rejected.
- **Person tracks:** a candidate becomes a person track after matching candidates in
  `person_min_frames` frames in a row. A track is dropped only after `person_clear_frames` frames
  in a row without a match, and until then it is reported with its last seen box and values.
  Candidates match a track when their columns overlap or are at most `merge_gap_cols` apart.
- **Fire:** turns on when any pixel is at or above `fire_min_c` in `fire_min_frames` frames in a
  row, and off after `fire_clear_frames` frames in a row with no pixel that hot. While on, the
  message carries the current frame's warmest pixel.
- **Bearing:** 0 = straight ahead, positive = the robot's left, 110 deg across the 32 columns
  (the driver's flips make the image's left the robot's left).

| Topic | Type | Content |
| --- | --- | --- |
| `/thermal/detections` | `hg_msgs/ThermalDetections` | Every frame: header of the image, room temperature, confirmed persons (pixel box, peak, degrees above room, bearing), fire flag with the frame's warmest pixel and its bearing |

The parameters and their defaults are in [`config/thermal.yaml`](config/thermal.yaml), each with
a comment, and are read once at start. The node logs only when the held state changes (the
number of person tracks, or fire on or off), so each real event gives one line:

```
person detected: 1, bearing 12 deg, +7.4 C above room
no person
FIRE: max 182.0 C at bearing -20 deg
fire cleared
```

`ros2 launch hg_perception thermal.launch.py` starts the driver and the detector; `hg_bringup`'s
`robot.launch.py` includes it, so both start with the robot.

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
the user must be in the `i2c` group (see
[docs/setup/raspberry-pi.md](../../../docs/setup/raspberry-pi.md)).

## License

Apache-2.0. The vendored Melexis MLX90640 API is also Apache-2.0; its license file is in
`third_party/mlx90640-library/LICENSE`.
