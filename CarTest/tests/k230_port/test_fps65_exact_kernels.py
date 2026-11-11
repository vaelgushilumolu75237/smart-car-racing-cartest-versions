"""Exact fresh-frame shortcuts and diagnostic bytes; no hardware claims."""
from pathlib import Path
from types import SimpleNamespace
import copy
import random
import struct
import sys
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'openmv'))
import config
import fast_topology
import k230_accel


class Fps65ExactKernels(unittest.TestCase):
    def test_threshold_planes_own_their_native_buffers(self):
        np = k230_accel.np
        shared = bytearray([0, 255, 0, 255])

        class NativePlane:
            def reshape(self, shape):
                return self

            def tobytes(self):
                return shared

            def __eq__(self, value):
                return np.frombuffer(shared, dtype=np.uint8).reshape((2, 2)) == value

        cv = SimpleNamespace(rgb888_erode=lambda *args: NativePlane())
        builder = SimpleNamespace(width=2, height=2, luma=bytearray([10, 200, 10, 200]),
                                  _rectangles=[0, 0, 2, 2], _block_count=1,
                                  _block_stats=[0] * 4)
        with patch.object(k230_accel, '_cv', cv):
            result = k230_accel.native_thresholds(builder)
        shared[:] = bytes(4)
        self.assertEqual(builder._native_binaries, [b'\x00\xff\x00\xff'])
        self.assertEqual(result[0][:3], (10, 2, 2))

    def test_signed_masks_match_general_path_and_clear_dirty_buffers(self):
        rng = random.Random(650031)
        for trial in range(120):
            rectangles = [0, 8, 80, 21, 0, 21, 80, 34,
                          0, 34, 80, 47, 0, 47, 80, 60]
            planes = [bytes(rng.choice((0, 255)) for _ in range(1040))
                      for _ in range(4)]
            builder = SimpleNamespace(
                width=80, height=60, _rectangles=rectangles, _block_count=4,
                _trusted_values=bytearray(
                    rng.randrange(2) if trial % 3 == 0 else 1 for _ in range(4)),
                _native_binaries=planes, road_mask=bytearray(rng.randbytes(4800)),
                edge_mask=bytearray(rng.randbytes(4800)),
                known_region=bytearray(rng.randbytes(4800)),
                road_row_counts=bytearray(60), row_counts=bytearray(60),
                _pixel_totals=[-1, -1])
            if trial % 7:
                builder._signed_binary_planes = builder._native_binaries
            if trial % 11 == 0:
                builder._rectangles[4] = 22  # Gap takes the general path.
                builder._native_binaries[1] = builder._native_binaries[1][80:]
            reference = copy.deepcopy(builder)
            self.assertEqual(k230_accel._native_masks_reference(reference),
                             k230_accel.native_masks(builder))
            for name in ('road_mask', 'edge_mask', 'known_region',
                         'road_row_counts', 'row_counts', '_pixel_totals'):
                self.assertEqual(getattr(reference, name), getattr(builder, name),
                                 (trial, name))

    def test_vertical_exterior_proof_matches_complete_island_checker(self):
        rng = random.Random(650032)
        islands = 0
        for trial in range(100):
            guard = fast_topology.FastTopologyGuard(config)
            floor = trial % 12
            mask = bytearray(4800)
            left, right = 8, 71
            for y in range(floor, 60):
                left = max(3, min(24, left + rng.choice((-1, 0, 1))))
                right = max(55, min(76, right + rng.choice((-1, 0, 1))))
                for x in (left, left + 1, right, right + 1):
                    mask[y * 80 + x] = 1
            if trial % 2:
                for y in range(24, 39):
                    for x in range(31, 46):
                        if y in (24, 38) or x in (31, 45):
                            mask[y * 80 + x] = 255
            guard._current_mask = mask
            guard._current_known = bytearray([1] * 4800)
            guard._current_floor = floor
            guard._current_band_end = 60
            other = copy.deepcopy(guard)
            expected = k230_accel._internal_component_reference(guard, floor, 60)
            self.assertEqual(expected, k230_accel.internal_component(other, floor, 60))
            islands += bool(expected)
        self.assertGreater(islands, 0)

    def test_vertical_proof_checks_every_row_and_preserves_internal_ring(self):
        guard = fast_topology.FastTopologyGuard(config)
        guard._current_mask = bytearray(4800)
        guard._current_known = bytearray([1] * 4800)
        guard._current_floor = 8
        guard._current_band_end = 60
        for y in range(22, 38):
            for x in range(30, 46):
                if x in (30, 45) or y in (22, 37):
                    guard._current_mask[y * 80 + x] = 1
        self.assertNotEqual(k230_accel.internal_component(guard, 8, 60), 0)
        guard._current_mask[:] = bytes(4800)
        self.assertEqual(k230_accel.internal_component(guard, 8, 60), 0)


if __name__ == '__main__':
    unittest.main()
