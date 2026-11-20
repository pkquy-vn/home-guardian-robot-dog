# SPDX-License-Identifier: Apache-2.0
"""False-color rendering of thermal images to PNG, standard library only.

Used by thermal_snapshot and meant to be reused by event alerts that attach thermal images.
No ROS imports: temperatures_from_image() reads any object with the sensor_msgs/Image fields.

Typical use:
    values, width, height = temperatures_from_image(msg)
    png, t_min, t_max = false_color_png(values, width, height, 320, 240)
"""

from __future__ import annotations

import math
import struct
import zlib
from typing import Optional, Sequence

# Ironbow-style palette, cold to hot: (position 0..1, (r, g, b)).
_IRONBOW_STOPS = (
    (0.00, (0, 0, 10)),
    (0.15, (40, 0, 110)),
    (0.30, (120, 0, 140)),
    (0.45, (190, 30, 90)),
    (0.60, (230, 80, 20)),
    (0.75, (250, 150, 0)),
    (0.90, (255, 220, 60)),
    (1.00, (255, 255, 220)),
)

# Smallest color span in deg C, so a nearly uniform scene is not stretched into pure noise.
MIN_COLOR_SPAN_C = 2.0

NAN_COLOR = (0, 0, 0)


def _build_lut(stops, size: int = 256) -> list[tuple[int, int, int]]:
    lut = []
    for i in range(size):
        x = i / (size - 1)
        for (x0, c0), (x1, c1) in zip(stops, stops[1:]):
            if x <= x1:
                f = (x - x0) / (x1 - x0)
                lut.append(tuple(round(a + (b - a) * f) for a, b in zip(c0, c1)))
                break
    return lut


IRONBOW = _build_lut(_IRONBOW_STOPS)


def temperatures_from_image(msg) -> tuple[list[float], int, int]:
    """Returns (row-major temperatures in deg C, width, height) from a 32FC1 sensor_msgs/Image."""
    if msg.encoding != "32FC1":
        raise ValueError(f"expected a 32FC1 image, got {msg.encoding!r}")
    width, height, step = int(msg.width), int(msg.height), int(msg.step)
    data = bytes(msg.data)
    if width <= 0 or height <= 0 or step < 4 * width or len(data) < step * height:
        raise ValueError(f"inconsistent image: {width}x{height}, step {step}, {len(data)} bytes")
    row_format = (">" if msg.is_bigendian else "<") + f"{width}f"
    values: list[float] = []
    for row in range(height):
        values.extend(struct.unpack_from(row_format, data, row * step))
    return values, width, height


def temperature_range(values: Sequence[float]) -> tuple[float, float]:
    """(min, max) of the finite values. Raises ValueError if there are none."""
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        raise ValueError("image has no finite temperatures")
    return min(finite), max(finite)


def _axis_samples(n_in: int, n_out: int) -> list[tuple[int, int, float]]:
    # Pixel-center aligned: output pixel i covers source position (i + 0.5) * n_in / n_out - 0.5.
    samples = []
    for i in range(n_out):
        src = min(max((i + 0.5) * n_in / n_out - 0.5, 0.0), n_in - 1.0)
        i0 = int(src)
        samples.append((i0, min(i0 + 1, n_in - 1), src - i0))
    return samples


def resize_bilinear(
    values: Sequence[float], width: int, height: int, out_width: int, out_height: int
) -> list[float]:
    """Bilinear resize of a row-major grid. NaN pixels spread to their neighbors."""
    xs = _axis_samples(width, out_width)
    out: list[float] = []
    for y0, y1, fy in _axis_samples(height, out_height):
        top = values[y0 * width : (y0 + 1) * width]
        bottom = values[y1 * width : (y1 + 1) * width]
        row = [a + (b - a) * fy for a, b in zip(top, bottom)]
        out.extend(row[x0] + (row[x1] - row[x0]) * fx for x0, x1, fx in xs)
    return out


def colorize(
    values: Sequence[float],
    t_min: float,
    t_max: float,
    palette: Sequence[tuple[int, int, int]] = IRONBOW,
) -> bytes:
    """RGB bytes for the values, mapping [t_min, t_max] onto the palette (cold to hot)."""
    span = t_max - t_min
    if span < MIN_COLOR_SPAN_C:
        center = (t_min + t_max) / 2.0
        t_min, span = center - MIN_COLOR_SPAN_C / 2.0, MIN_COLOR_SPAN_C
    top = len(palette) - 1
    rgb = bytearray()
    for v in values:
        if math.isfinite(v):
            index = min(max(int((v - t_min) / span * top + 0.5), 0), top)
            rgb.extend(palette[index])
        else:
            rgb.extend(NAN_COLOR)
    return bytes(rgb)


def encode_png(rgb: bytes, width: int, height: int) -> bytes:
    """Minimal 8-bit RGB PNG encoder."""
    if len(rgb) != width * height * 3:
        raise ValueError("rgb size does not match width x height x 3")
    stride = width * 3
    raw = b"".join(b"\x00" + rgb[y * stride : (y + 1) * stride] for y in range(height))

    def chunk(tag: bytes, payload: bytes) -> bytes:
        crc = zlib.crc32(tag + payload) & 0xFFFFFFFF
        return struct.pack(">I", len(payload)) + tag + payload + struct.pack(">I", crc)

    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)  # 8-bit truecolor
    return (
        b"\x89PNG\r\n\x1a\n"
        + chunk(b"IHDR", header)
        + chunk(b"IDAT", zlib.compress(raw, 6))
        + chunk(b"IEND", b"")
    )


def false_color_png(
    values: Sequence[float],
    width: int,
    height: int,
    out_width: int = 320,
    out_height: int = 240,
    t_min: Optional[float] = None,
    t_max: Optional[float] = None,
) -> tuple[bytes, float, float]:
    """Renders temperatures as a false-color PNG, scaled to out_width x out_height.

    The color scale runs from t_min to t_max (default: the image's own min and max). Returns
    (png_bytes, image_min, image_max), with min and max taken from the unscaled temperatures.
    Row 0 is the top of the image and column 0 the left, as published.
    """
    image_min, image_max = temperature_range(values)
    lo = image_min if t_min is None else t_min
    hi = image_max if t_max is None else t_max
    if (out_width, out_height) != (width, height):
        values = resize_bilinear(values, width, height, out_width, out_height)
    rgb = colorize(values, lo, hi)
    return encode_png(rgb, out_width, out_height), image_min, image_max
