# SPDX-License-Identifier: Apache-2.0
"""Simple person and fire detection on a thermal image, standard library only (no ROS).

Used by the thermal_detector node. Image layout as published on /thermal/image: row-major,
row 0 at the top, column 0 at the image's left (the robot's left).

Person: a pixel is person-warm if it is at least person_delta_c above the room temperature (median
of all pixels) and at most person_max_c. Touching person-warm pixels (8-neighbour) form clusters;
clusters with at most merge_gap_cols empty columns between their column ranges are merged (a close
person shows as two legs, because the camera is low). A merged cluster is a candidate if it has at
least person_min_pixels pixels and is at least person_min_aspect times taller (rows) than wide
(columns).

Person tracks: a candidate becomes a confirmed person track after matching candidates in
person_min_frames consecutive frames. A track is dropped only after person_clear_frames
consecutive frames without a match; until then it is reported with its last seen box and values.
Candidates match tracks when their column ranges overlap or are at most merge_gap_cols empty
columns apart (movement between frames).

Fire: turns on when a pixel is at or above fire_min_c in fire_min_frames consecutive frames, and
off when no pixel has reached fire_min_c for fire_clear_frames consecutive frames.
"""

from __future__ import annotations

import math
import statistics
from dataclasses import dataclass
from typing import Callable, Sequence

HORIZONTAL_FOV_DEG = 110.0  # MLX90640 110 x 75 deg version: 110 deg across the 32 columns.


@dataclass
class DetectorParams:
    person_delta_c: float = 3.0
    person_max_c: float = 40.0
    person_min_pixels: int = 3
    person_min_aspect: float = 1.5
    person_min_frames: int = 3
    person_clear_frames: int = 8
    merge_gap_cols: int = 3
    fire_min_c: float = 100.0
    fire_min_frames: int = 8
    fire_clear_frames: int = 8


@dataclass
class Cluster:
    col_min: int
    col_max: int
    row_min: int
    row_max: int
    pixels: int
    peak_c: float

    @property
    def width(self) -> int:
        return self.col_max - self.col_min + 1

    @property
    def height(self) -> int:
        return self.row_max - self.row_min + 1

    @property
    def center_col(self) -> float:
        return (self.col_min + self.col_max) / 2.0

    def merged(self, other: Cluster) -> Cluster:
        return Cluster(
            col_min=min(self.col_min, other.col_min),
            col_max=max(self.col_max, other.col_max),
            row_min=min(self.row_min, other.row_min),
            row_max=max(self.row_max, other.row_max),
            pixels=self.pixels + other.pixels,
            peak_c=max(self.peak_c, other.peak_c),
        )


@dataclass
class Person:
    box: Cluster
    peak_c: float
    delta_c: float  # peak_c minus room temperature, when last seen
    bearing_deg: float


@dataclass
class Fire:
    active: bool
    max_c: float  # Warmest pixel of the current frame, also when not active.
    col: int
    row: int
    bearing_deg: float


@dataclass
class Detections:
    room_c: float
    persons: list[Person]  # Confirmed tracks, including those held without a match this frame.
    fire: Fire


def bearing_deg(col: float, width: int, fov_deg: float = HORIZONTAL_FOV_DEG) -> float:
    """Bearing of a (fractional) column: 0 = image center (straight ahead), positive = left."""
    return ((width - 1) / 2.0 - col) * fov_deg / width


def room_temperature(values: Sequence[float]) -> float:
    """Median of the finite pixel values. Raises ValueError if there are none."""
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        raise ValueError("image has no finite temperatures")
    return statistics.median(finite)


def find_clusters(
    values: Sequence[float], width: int, height: int, is_warm: Callable[[float], bool]
) -> list[Cluster]:
    """8-neighbour connected groups of pixels for which is_warm() is true."""
    warm = [is_warm(v) for v in values]
    seen = [False] * len(values)
    clusters = []
    for start, start_warm in enumerate(warm):
        if not start_warm or seen[start]:
            continue
        seen[start] = True
        stack = [start]
        col_min, row_min, col_max, row_max = width, height, -1, -1
        pixels = 0
        peak = -math.inf
        while stack:
            index = stack.pop()
            row, col = divmod(index, width)
            col_min, col_max = min(col_min, col), max(col_max, col)
            row_min, row_max = min(row_min, row), max(row_max, row)
            pixels += 1
            peak = max(peak, values[index])
            for d_row in (-1, 0, 1):
                for d_col in (-1, 0, 1):
                    r, c = row + d_row, col + d_col
                    if (d_row or d_col) and 0 <= r < height and 0 <= c < width:
                        neighbour = r * width + c
                        if warm[neighbour] and not seen[neighbour]:
                            seen[neighbour] = True
                            stack.append(neighbour)
        clusters.append(Cluster(col_min, col_max, row_min, row_max, pixels, peak))
    return clusters


def column_gap(a: Cluster, b: Cluster) -> int:
    """Empty columns between two column ranges: 0 if adjacent, negative if they overlap."""
    return max(a.col_min, b.col_min) - min(a.col_max, b.col_max) - 1


def merge_by_columns(clusters: Sequence[Cluster], max_gap_cols: int) -> list[Cluster]:
    """Merges clusters whose column ranges have at most max_gap_cols empty columns between them."""
    merged: list[Cluster] = []
    for cluster in sorted(clusters, key=lambda c: c.col_min):
        if merged and cluster.col_min - merged[-1].col_max - 1 <= max_gap_cols:
            merged[-1] = merged[-1].merged(cluster)
        else:
            merged.append(cluster)
    return merged


@dataclass
class _Track:
    person: Person  # Last matched candidate.
    hits: int = 1  # Consecutive matched frames (counts up to confirmation).
    misses: int = 0  # Consecutive frames without a match (confirmed tracks only).


def _match(
    tracks: Sequence[_Track], candidates: Sequence[Person], max_gap_cols: int
) -> list[tuple[int, int]]:
    """One-to-one (track index, candidate index) pairs, closest column ranges first."""
    pairs = []
    for t, track in enumerate(tracks):
        for c, candidate in enumerate(candidates):
            gap = column_gap(track.person.box, candidate.box)
            if gap <= max_gap_cols:
                pairs.append((gap, -candidate.box.pixels, t, c))
    pairs.sort()
    used_tracks: set[int] = set()
    used_candidates: set[int] = set()
    matches = []
    for _, _, t, c in pairs:
        if t not in used_tracks and c not in used_candidates:
            used_tracks.add(t)
            used_candidates.add(c)
            matches.append((t, c))
    return matches


class PersonTracker:
    """Confirms candidates over min_frames frames and holds confirmed tracks for clear_frames."""

    def __init__(self, min_frames: int, clear_frames: int, max_gap_cols: int) -> None:
        self._min_frames = max(1, min_frames)
        self._clear_frames = max(1, clear_frames)
        self._max_gap_cols = max_gap_cols
        self._confirmed: list[_Track] = []
        self._pending: list[_Track] = []

    def update(self, candidates: Sequence[Person]) -> list[Person]:
        """Feeds one frame's candidates; returns the confirmed persons, left to right."""
        # Confirmed tracks get the first pick of the candidates.
        matched_tracks = set()
        used = set()
        for t, c in _match(self._confirmed, candidates, self._max_gap_cols):
            self._confirmed[t].person = candidates[c]
            self._confirmed[t].misses = 0
            matched_tracks.add(t)
            used.add(c)
        for t, track in enumerate(self._confirmed):
            if t not in matched_tracks:
                track.misses += 1
        self._confirmed = [t for t in self._confirmed if t.misses < self._clear_frames]

        # The rest extend pending tracks (which must match every frame) or start new ones.
        remaining = [p for c, p in enumerate(candidates) if c not in used]
        pending = []
        matched_remaining = set()
        for t, c in _match(self._pending, remaining, self._max_gap_cols):
            track = self._pending[t]
            track.person = remaining[c]
            track.hits += 1
            pending.append(track)
            matched_remaining.add(c)
        pending.extend(
            _Track(person) for c, person in enumerate(remaining) if c not in matched_remaining
        )

        self._pending = []
        for track in pending:
            if track.hits >= self._min_frames:
                track.misses = 0
                self._confirmed.append(track)
            else:
                self._pending.append(track)
        return sorted((t.person for t in self._confirmed), key=lambda p: p.box.col_min)


class FireMonitor:
    """On after min_frames consecutive hot frames, off after clear_frames consecutive cool ones.

    A frame is hot when its warmest pixel is at or above min_c.
    """

    def __init__(self, min_c: float, min_frames: int, clear_frames: int) -> None:
        self._min_c = min_c
        self._min_frames = max(1, min_frames)
        self._clear_frames = max(1, clear_frames)
        self._hot_frames = 0
        self._cool_frames = 0
        self.active = False

    def update(self, max_c: float) -> bool:
        if max_c >= self._min_c:
            self._hot_frames += 1
            self._cool_frames = 0
        else:
            self._cool_frames += 1
            self._hot_frames = 0
        if not self.active and self._hot_frames >= self._min_frames:
            self.active = True
        elif self.active and self._cool_frames >= self._clear_frames:
            self.active = False
        return self.active


class ThermalDetector:
    """Runs person and fire detection frame by frame. Keeps the state between frames."""

    def __init__(self, params: DetectorParams) -> None:
        self.params = params
        self._tracker = PersonTracker(
            params.person_min_frames, params.person_clear_frames, params.merge_gap_cols
        )
        self._fire = FireMonitor(
            params.fire_min_c, params.fire_min_frames, params.fire_clear_frames
        )

    def process(self, values: Sequence[float], width: int, height: int) -> Detections:
        """Detects persons and fire in one image. Raises ValueError for an all-NaN image."""
        p = self.params
        room = room_temperature(values)

        low, high = room + p.person_delta_c, p.person_max_c
        clusters = find_clusters(values, width, height, lambda t: low <= t <= high)
        candidates = [
            Person(c, c.peak_c, c.peak_c - room, bearing_deg(c.center_col, width))
            for c in merge_by_columns(clusters, p.merge_gap_cols)
            if c.pixels >= p.person_min_pixels and c.height >= p.person_min_aspect * c.width
        ]
        persons = self._tracker.update(candidates)

        hottest = max(
            (i for i, v in enumerate(values) if math.isfinite(v)), key=lambda i: values[i]
        )
        row, col = divmod(hottest, width)
        max_c = values[hottest]
        fire = Fire(self._fire.update(max_c), max_c, col, row, bearing_deg(col, width))
        return Detections(room, persons, fire)
