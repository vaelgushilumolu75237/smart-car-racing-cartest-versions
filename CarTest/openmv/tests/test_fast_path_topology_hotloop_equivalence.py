"""Independent checks for the V7 path-record and topology hot-loop changes."""

import random
import importlib.util
import sys
import unittest
from pathlib import Path


OPENMV = Path(__file__).resolve().parents[1]
if str(OPENMV) not in sys.path:
    sys.path.insert(0, str(OPENMV))

import config
import fast_path
import fast_topology


_REFERENCE_PATH = (Path(__file__).resolve().parent /
                   "hotloop_reference_20261023" / "fast_path.py")
_REFERENCE_SPEC = importlib.util.spec_from_file_location(
    "fast_path_pre_hotloop_20261023", _REFERENCE_PATH)
REFERENCE_FAST_PATH = importlib.util.module_from_spec(_REFERENCE_SPEC)
_REFERENCE_SPEC.loader.exec_module(REFERENCE_FAST_PATH)
_REFERENCE_TOPOLOGY_SPEC = importlib.util.spec_from_file_location(
    "fast_topology_pre_hotloop_20261023",
    _REFERENCE_PATH.with_name("fast_topology.py"))
REFERENCE_FAST_TOPOLOGY = importlib.util.module_from_spec(
    _REFERENCE_TOPOLOGY_SPEC)
_REFERENCE_TOPOLOGY_SPEC.loader.exec_module(REFERENCE_FAST_TOPOLOGY)
# The preserved runtime's CPython shim predates direct host execution of its
# Viper body. Identity adapters let this test call that exact kernel body.
REFERENCE_FAST_TOPOLOGY.ptr8 = lambda buffer: buffer
REFERENCE_FAST_TOPOLOGY.ptr16 = lambda buffer: buffer


WIDTH = 80
HEIGHT = 60
TOTAL = WIDTH * HEIGHT


def _road_info(center_fn=None, half_width=16, extras=(), y_start=None,
               unknown_rows=()):
    if y_start is None:
        y_start = config.SURFACE_ROI_FLOOR_Y
    if center_fn is None:
        center_fn = lambda _y: 39.5
    floor = config.SURFACE_ROI_FLOOR_Y
    luma = bytearray([28]) * TOTAL
    road = bytearray(TOTAL)
    edge = bytearray(TOTAL)
    known = bytearray([1]) * TOTAL
    for y in range(y_start, HEIGHT):
        row = y * WIDTH
        center = center_fn(y)
        runs = [(int(center - half_width + 0.5),
                 int(center + half_width + 0.5))]
        runs.extend(extras)
        for left, right in runs:
            left = max(1, left)
            right = min(WIDTH - 2, right)
            for x in range(left, right + 1):
                road[row + x] = 1
                luma[row + x] = 220
            edge[row + left - 1] = 1
            edge[row + right + 1] = 1
        if y in unknown_rows:
            known[row:row + WIDTH] = bytes(WIDTH)
            edge[row:row + WIDTH] = bytes(WIDTH)
            road[row:row + WIDTH] = bytes(WIDTH)
            luma[row:row + WIDTH] = bytes(WIDTH)
    return {
        "valid": True,
        "width": WIDTH,
        "height": HEIGHT,
        "floor": floor,
        "luma": luma,
        "road_mask": road,
        "edge_mask": edge,
        "known_region": known,
        "row_counts": bytearray(
            sum(edge[y * WIDTH:(y + 1) * WIDTH]) for y in range(HEIGHT)),
        "road_row_counts": bytearray(
            sum(road[y * WIDTH:(y + 1) * WIDTH])
            for y in range(HEIGHT)),
    }


def _merge_after_reference_info():
    """Adapt the two routes that split then merge to the active floor of 8."""
    luma = bytearray([28]) * TOTAL
    road = bytearray(TOTAL)
    edge = bytearray(TOTAL)
    known = bytearray([1]) * TOTAL

    def add_run(y, left, right):
        row = y * WIDTH
        for x in range(left, right + 1):
            road[row + x] = 1
            luma[row + x] = 220
        edge[row + left - 1] = 1
        edge[row + right + 1] = 1

    for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT):
        if y >= 49:
            add_run(y, 36, 43)
        elif y >= 40:
            t = (48 - y) / 8.0
            left_center = 35.0 - 15.0 * t
            right_center = 44.0 + 15.0 * t
            left = int(left_center - 3.5 + 0.5)
            right = int(right_center - 3.5 + 0.5)
            add_run(y, left, left + 7)
            add_run(y, right, right + 7)
        elif y >= 24:
            t = (39 - y) / 15.0
            left_center = 22.0 + 13.0 * t
            right_center = 57.0 - 13.0 * t
            left = int(left_center - 3.5 + 0.5)
            right = int(right_center - 3.5 + 0.5)
            add_run(y, left, left + 7)
            add_run(y, right, right + 7)
        else:
            add_run(y, 31, 48)
    return {
        "valid": True,
        "width": WIDTH,
        "height": HEIGHT,
        "floor": config.SURFACE_ROI_FLOOR_Y,
        "luma": luma,
        "road_mask": road,
        "edge_mask": edge,
        "known_region": known,
        "row_counts": bytearray(
            sum(edge[y * WIDTH:(y + 1) * WIDTH]) for y in range(HEIGHT)),
        "road_row_counts": bytearray(
            sum(road[y * WIDTH:(y + 1) * WIDTH])
            for y in range(HEIGHT)),
    }


def _path_public_summary(result):
    scalar_keys = (
        "valid", "reason", "scene", "quality", "lateral_error",
        "reference_y", "reference_target_y", "reference_shift_cells",
        "center_x", "reference_center_x", "reference_provenance",
        "reference_lateral_error", "observed_rows", "active_row_count",
        "selected_row_count", "selected_row_stride", "candidate_tracks",
        "track_score", "vertical_span", "weight_source", "flags",
    )
    summary = {key: result.get(key) for key in scalar_keys}
    # The path result has no flags field; at the current main.py integration
    # point the packet's valid bit is derived directly from this value and the
    # slope-hint bit is passed as zero.
    summary["flags"] = result.get(
        "flags", 1 if result.get("valid") else 0)
    summary["selected_rows"] = tuple(
        (row["y"], row["left"], row["right"], row["center"],
         row["width"], row["pixels"], row["observed"],
         row["left_status"], row["right_status"], row["provenance"],
         tuple(row["boundary"]["left"]),
         tuple(row["boundary"]["right"]))
        for row in result["selected_rows"])
    summary["selected_row_ints"] = tuple(result["selected_row_ints"])
    topology = result.get("topology_evidence")
    summary["ambiguity"] = (
        bool(topology.get("ambiguous")) if isinstance(topology, dict) else
        result.get("scene") == "AMBIGUOUS_CANDIDATE")
    summary["topology_evidence"] = (
        tuple(sorted(topology.items())) if isinstance(topology, dict) else
        topology)
    return summary


def _viper_mask(seed, density):
    rng = random.Random(seed)
    mask = bytearray(TOTAL)
    for y in range(20, HEIGHT):
        offset = y * WIDTH
        for x in range(WIDTH):
            mask[offset + x] = int(rng.random() < density)
    return mask


def _bind_mask(guard, mask):
    guard._current_mask = mask
    guard._current_floor = 20
    guard._current_band_end = HEIGHT


def _region_inputs(unknown_rows=(), partial_rows=(), bad_row=None):
    info = _road_info(unknown_rows=unknown_rows)
    for y in partial_rows:
        info["known_region"][y * WIDTH] = 0
    if bad_row is not None:
        info["row_counts"][bad_row] = (
            info["row_counts"][bad_row] + 1) & 0xFF
    return info


def _bind_region(guard, info):
    guard._current_mask = info["edge_mask"]
    guard._current_known = info["known_region"]
    guard._current_rows = info["row_counts"]


class CandidateRecordEquivalenceTests(unittest.TestCase):
    def test_original_four_row_eight_span_support_floor_is_preserved(self):
        detector = fast_path.FastPathDetector(config)
        self.assertEqual(detector.min_rows, 4)
        self.assertEqual(detector.min_span, 8)
        with self.assertRaises(ValueError):
            fast_path.FastPathDetector({"FAST_PATH_MIN_OBSERVED_ROWS": 3})
        with self.assertRaises(ValueError):
            fast_path.FastPathDetector({"FAST_PATH_MIN_VERTICAL_SPAN": 7})

    def test_extracted_candidate_records_match_independent_scan(self):
        detector = fast_path.FastPathDetector(config)
        width = detector.width
        height = detector.height
        floor = detector.floor_y
        luma = bytearray([28]) * (width * height)
        road = bytearray(width * height)
        edge = bytearray(width * height)
        known = bytearray([1]) * (width * height)

        # Exercise multiple runs, missing rows, odd/even widths, and variable
        # pixel values while keeping all edge witnesses explicit.
        for y in range(floor, height):
            if y % 5 == 1:
                continue
            runs = [(11 + y % 3, 21 + y % 3)]
            if y % 2 == 0:
                runs.append((55, 64 + y % 2))
            row = y * width
            for left, right in runs:
                for x in range(left, right + 1):
                    road[row + x] = 1
                    luma[row + x] = 170 + ((x * 7 + y * 11) % 86)
                edge[row + left - 1] = 1
                edge[row + right + 1] = 1

        detector._current_luma = luma
        detector._current_road = road
        detector._current_edge = edge
        detector._current_known = known
        self.assertEqual(fast_path._extract_viper(detector), 0)

        # This deliberately uses a separate row scanner and derives record
        # fields from pixel runs, rather than sharing the optimized layout.
        expected = []
        for layer in range(detector.row_count):
            y = height - 1 - layer
            row = y * width
            x = 0
            row_candidates = []
            while x < width:
                index = row + x
                if not known[index] or not road[index]:
                    x += 1
                    continue
                left = x
                bright_sum = 0
                while x < width:
                    index = row + x
                    if not known[index] or not road[index]:
                        break
                    bright_sum += luma[index]
                    x += 1
                right = x - 1
                run_width = right - left + 1
                left_hit = left - 1
                right_hit = right + 1
                if (run_width < detector.min_width or
                        run_width > detector.max_width or left_hit < 0 or
                        right_hit >= width):
                    continue
                left_index = row + left_hit
                right_index = row + right_hit
                if (not known[left_index] or not known[right_index] or
                        road[left_index] or road[right_index] or
                        not edge[left_index] or not edge[right_index]):
                    continue
                contrast = (bright_sum * 1000 // run_width -
                            (luma[left_index] + luma[right_index]) * 500)
                row_candidates.append((left, right, run_width, contrast))
            expected.append(row_candidates)

        for layer, row_candidates in enumerate(expected):
            self.assertEqual(detector._counts[layer], len(row_candidates))
            for slot, (left, right, run_width, contrast) in enumerate(
                    row_candidates):
                base = (layer * detector.max_per_row + slot) * 4
                self.assertEqual(detector._candidate_data[base], left)
                self.assertEqual(detector._candidate_data[base + 1],
                                 left + right)
                self.assertEqual(detector._candidate_data[base + 2],
                                 run_width)
                self.assertEqual(detector._candidate_data[base + 3], contrast)
                self.assertEqual(
                    detector._candidate_data[base] +
                    detector._candidate_data[base + 2] - 1, right)

    def test_full_detector_matches_pre_hotloop_source_at_active_floor(self):
        self.assertEqual(config.SURFACE_ROI_FLOOR_Y, 8)
        cases = (
            ("straight", _road_info(), (True, "OK", "ROAD_CANDIDATE")),
            ("left_curve", _road_info(
                center_fn=lambda y: 30.0 + 0.20 * (y - 20)),
             (True, "OK", "ROAD_CANDIDATE")),
            ("right_curve", _road_info(
                center_fn=lambda y: 49.0 - 0.20 * (y - 20)),
             (True, "OK", "ROAD_CANDIDATE")),
            ("competing_corridors", _road_info(
                center_fn=lambda _y: 39.5, half_width=12,
                extras=((4, 15), (64, 75))),
             (False, "AMBIGUOUS_ROAD", "AMBIGUOUS_CANDIDATE")),
            ("split_then_merge", _merge_after_reference_info(),
             (False, "AMBIGUOUS_ROAD", "AMBIGUOUS_CANDIDATE")),
            ("insufficient_rows", _road_info(y_start=58),
             (False, "INSUFFICIENT_FRESH_ROWS", "UNKNOWN")),
            ("unknown_band", _road_info(unknown_rows=(31, 32, 33)),
             (True, "OK", "ROAD_CANDIDATE")),
        )
        for label, info, expected_status in cases:
            with self.subTest(case=label):
                before = REFERENCE_FAST_PATH.FastPathDetector(config)
                after = fast_path.FastPathDetector(config)
                old_result = before.detect(info)
                new_result = after.detect(info)
                self.assertEqual((old_result["valid"], old_result["reason"],
                                  old_result["scene"]), expected_status)
                self.assertEqual(_path_public_summary(new_result),
                                 _path_public_summary(old_result))
                # detect() runs the Viper-decorated extract, connect, and
                # select kernels through the host shim; equal state/control
                # planes prove the center-cache representation reached all
                # candidate links and ambiguity decisions identically.
                self.assertEqual(tuple(after._counts), tuple(before._counts))
                self.assertEqual(tuple(after._state_data),
                                 tuple(before._state_data))
                self.assertEqual(tuple(after._control), tuple(before._control))

    def test_lazy_row_view_keeps_exposed_row_and_boundary_aliases_current(self):
        detector = fast_path.FastPathDetector(config)
        first = _road_info()
        result = detector.detect(first)
        self.assertTrue(result["valid"], result["reason"])
        rows = result["selected_rows"]

        retained = rows[0]
        retained_boundary = retained["boundary"]
        retained_left = retained_boundary["left"]
        retained_right = retained_boundary["right"]
        inactive = rows[35]
        inactive_snapshot = (
            inactive["y"], inactive["left"], inactive["right"],
            tuple(inactive["boundary"]["left"]),
            tuple(inactive["boundary"]["right"]))

        second = _road_info(center_fn=lambda _y: 52.5, y_start=30)
        # Keep only thirty observed rows so a previously exposed record outside
        # the new path retains its last-frame values, as it did before lazy
        # materialization.

        recovered = detector.detect(second)
        self.assertIs(recovered, result)
        self.assertTrue(recovered["valid"], recovered["reason"])
        self.assertEqual(len(rows), 30)
        current = rows[0]
        self.assertIs(current, retained)
        self.assertIs(current["boundary"], retained_boundary)
        self.assertIs(current["boundary"]["left"], retained_left)
        self.assertIs(current["boundary"]["right"], retained_right)
        packed = recovered["selected_row_ints"]
        self.assertEqual((retained["y"], retained["left"],
                          retained["right"]), tuple(packed[:3]))
        self.assertEqual(retained["center"],
                         (packed[1] + packed[2]) * 0.5)
        self.assertEqual(retained["width"], packed[2] - packed[1] + 1)
        self.assertEqual(retained["pixels"], retained["width"])
        self.assertEqual(retained_left, [packed[1] - 1, packed[0]])
        self.assertEqual(retained_right, [packed[2] + 1, packed[0]])

        # This row was never exposed before the second frame. Its first view
        # access materializes the current packed record, and its aliases then
        # follow the same refresh rule on later frames.
        newly_exposed = rows[6]
        offset = 6 * recovered["selected_row_stride"]
        self.assertEqual((newly_exposed["y"], newly_exposed["left"],
                          newly_exposed["right"]),
                         tuple(packed[offset:offset + 3]))
        self.assertEqual(inactive_snapshot, (
            inactive["y"], inactive["left"], inactive["right"],
            tuple(inactive["boundary"]["left"]),
            tuple(inactive["boundary"]["right"])))

    def test_recovery_packer_rejects_corrupt_counts_and_trace_nodes(self):
        detector = fast_path.FastPathDetector(config)
        invalid_states = (
            ("empty_path", 0, None, None),
            ("oversized_path", detector.row_count + 1, None, None),
            ("negative_node", 1, -1, None),
            ("node_past_candidate_capacity", 1,
             detector.row_count * detector.max_per_row, None),
            ("short_trace", 2, 0, "trace"),
            ("short_output", 2, 0, "output"),
            ("short_candidates", 1, 1, "candidates"),
            ("zero_max_per_row", 1, 0, "max_per_row"),
        )
        for label, path_count, node, mutation in invalid_states:
            with self.subTest(state=label):
                detector = fast_path.FastPathDetector(config)
                detector._control[3] = path_count
                if node is not None:
                    detector._trace[0] = node
                if mutation == "trace":
                    detector._trace = detector._trace[:1]
                elif mutation == "output":
                    detector.selected_row_ints = (
                        detector.selected_row_ints[:4])
                elif mutation == "candidates":
                    detector._candidate_data = (
                        detector._candidate_data[:4])
                elif mutation == "max_per_row":
                    detector.max_per_row = 0

                result = detector._recover_output()

                self.assertFalse(result["valid"])
                self.assertEqual(result["reason"], "CORRUPT_PATH_STATE")
                self.assertEqual(result["selected_row_count"], 0)
                self.assertEqual(len(result["selected_rows"]), 0)


class TopologyEpochEquivalenceTests(unittest.TestCase):
    def _assert_viper_matches_pixel_reference(self, mask, guard=None):
        if guard is None:
            guard = fast_topology.FastTopologyGuard(config)
        reference = fast_topology.FastTopologyGuard(config)
        _bind_mask(guard, mask)
        _bind_mask(reference, mask)
        expected = reference._internal_component_host(20, HEIGHT)
        actual = fast_topology._internal_component_viper(guard, 20, HEIGHT)
        self.assertEqual(actual, expected)
        return guard

    def test_scanline_zero_fill_preserves_closed_open_and_small_holes(self):
        def outline(x0, y0, x1, y1, gap=None, interior_dark=()):
            mask = bytearray(TOTAL)
            for y in range(y0, y1 + 1):
                for x in range(x0, x1 + 1):
                    boundary = (x == x0 or x == x1 or
                                y == y0 or y == y1)
                    if boundary and (x, y) != gap:
                        mask[y * WIDTH + x] = 1
            for x, y in interior_dark:
                mask[y * WIDTH + x] = 1
            return mask

        cases = (
            # A 4x4 outline encloses exactly the configured four-pixel
            # minimum. This exercises the threshold edge as well as filling.
            ("threshold_hole", outline(30, 35, 33, 38), 1),
            # Three connected interior zero pixels remain below the threshold
            # even though the surrounding dark component passes size checks.
            ("undersized_hole", outline(
                30, 35, 34, 39,
                interior_dark=((33, 36), (32, 37), (33, 37),
                               (31, 38), (32, 38), (33, 38))), 0),
            # The opening reconnects the interior zero area to the padded-box
            # border under the required 4-connectivity.
            ("open_outline", outline(30, 35, 34, 39, gap=(32, 35)), 0),
        )
        for label, mask, expected in cases:
            with self.subTest(case=label):
                reference = fast_topology.FastTopologyGuard(config)
                optimized = fast_topology.FastTopologyGuard(config)
                _bind_mask(reference, mask)
                _bind_mask(optimized, mask)
                reference_result = reference._internal_component_host(
                    20, HEIGHT)
                actual = fast_topology._internal_component_viper(
                    optimized, 20, HEIGHT)
                self.assertEqual(reference_result, expected)
                self.assertEqual(actual, reference_result)

    def test_epoch_kernel_matches_preserved_pre_hotloop_viper(self):
        cases = tuple((0x71A0 + index, density)
                      for index, density in enumerate(
                          (0.01, 0.08, 0.2, 0.45) * 4))
        for index, (seed, density) in enumerate(cases):
            with self.subTest(seed=seed, density=density):
                mask = _viper_mask(seed, density)
                before = REFERENCE_FAST_TOPOLOGY.FastTopologyGuard(config)
                after = fast_topology.FastTopologyGuard(config)
                _bind_mask(before, mask)
                _bind_mask(after, mask)
                stamp = 0xA5
                before._visited[:] = bytes([stamp]) * TOTAL
                after._visited[:] = bytes([stamp]) * TOTAL
                if index == len(cases) - 1:
                    after._visit_epoch[0] = 63
                expected = (
                    REFERENCE_FAST_TOPOLOGY._internal_component_viper(
                        before, 20, HEIGHT))
                actual = fast_topology._internal_component_viper(
                    after, 20, HEIGHT)
                self.assertEqual(actual, expected)
                if index == len(cases) - 1:
                    self.assertEqual(after._visit_epoch[0], 1)

    def test_seeded_masks_match_pixel_bfs_with_stale_visited_bytes(self):
        guard = fast_topology.FastTopologyGuard(config)
        for index, density in enumerate((0.02, 0.08, 0.2, 0.45)):
            guard._visit_epoch[0] = 10 + index
            stale_stamp = ((int(guard._visit_epoch[0]) << 2) | 2) & 0xFF
            guard._visited[:] = bytes([stale_stamp]) * TOTAL
            mask = _viper_mask(0x5A00 + index, density)
            guard = self._assert_viper_matches_pixel_reference(mask, guard)
            self.assertEqual(guard._visit_epoch[0], 11 + index)

    def test_epoch_wrap_clears_stale_bytes_before_reuse(self):
        mask = _viper_mask(0xE0C4, 0.18)
        guard = fast_topology.FastTopologyGuard(config)
        guard._visit_epoch[0] = 63
        guard._visited[:] = bytes([0xFD]) * TOTAL
        _bind_mask(guard, mask)
        expected_guard = fast_topology.FastTopologyGuard(config)
        _bind_mask(expected_guard, mask)
        expected = expected_guard._internal_component_host(20, HEIGHT)

        actual = fast_topology._internal_component_viper(guard, 20, HEIGHT)

        self.assertEqual(actual, expected)
        self.assertEqual(guard._visit_epoch[0], 1)
        # Row zero lies outside the ROI passed to the kernel; it can only be
        # zero now because the wrap path cleared the complete visited map.
        self.assertEqual(guard._visited[0], 0)

    def test_successive_masks_advance_epoch_without_reusing_old_marks(self):
        guard = fast_topology.FastTopologyGuard(config)
        for index, density in enumerate((0.22, 0.01, 0.31, 0.05) * 4):
            mask = _viper_mask(0x9000 + index, density)
            guard = self._assert_viper_matches_pixel_reference(mask, guard)
            self.assertEqual(guard._visit_epoch[0], index + 1)


class RegionScanEquivalenceTests(unittest.TestCase):
    def test_region_scan_outputs_and_trusted_row_count_match_host(self):
        cases = (
            ("all_known", _region_inputs(), 1),
            ("unknown_bands", _region_inputs(unknown_rows=(17, 18, 41)), 0),
            ("mixed_known_row", _region_inputs(partial_rows=(29,)), -2),
            ("row_count_mismatch", _region_inputs(bad_row=33), -1),
        )
        for label, info, expected_status in cases:
            with self.subTest(case=label):
                host = fast_topology.FastTopologyGuard(config)
                optimized = fast_topology.FastTopologyGuard(config)
                _bind_region(host, info)
                _bind_region(optimized, info)
                # Prove the kernel overwrites stale trust bits across its ROI.
                host._trusted_rows[:] = bytes([1]) * HEIGHT
                optimized._trusted_rows[:] = bytes([1]) * HEIGHT

                expected = host._region_segments_host(
                    config.SURFACE_ROI_FLOOR_Y)
                actual = fast_topology._region_segments_viper(
                    optimized, config.SURFACE_ROI_FLOOR_Y)
                self.assertEqual(expected, expected_status)
                self.assertEqual(actual, expected)
                self.assertEqual(optimized._segment_count_buffer[0],
                                 host._segment_count_buffer[0])
                if actual >= 0:
                    trusted_count = sum(
                        1 for y in range(config.SURFACE_ROI_FLOOR_Y, HEIGHT)
                        if all(info["known_region"][
                            y * WIDTH:(y + 1) * WIDTH]))
                    self.assertEqual(optimized._segment_count_buffer[1],
                                     trusted_count)
                    self.assertEqual(optimized._trusted_rows[
                        config.SURFACE_ROI_FLOOR_Y:], host._trusted_rows[
                            config.SURFACE_ROI_FLOOR_Y:])
                else:
                    self.assertEqual(optimized._segment_count_buffer[1], 0)

    def test_viper_wrapper_result_matches_host_topology_evidence(self):
        cases = (
            _region_inputs(),
            _region_inputs(unknown_rows=(17, 18, 41)),
            _region_inputs(partial_rows=(29,)),
        )
        old_backend = fast_topology._IS_MICROPYTHON
        try:
            fast_topology._IS_MICROPYTHON = False
            expected = [fast_topology.FastTopologyGuard(config).check(info)
                        .copy() for info in cases]
            fast_topology._IS_MICROPYTHON = True
            actual = [fast_topology.FastTopologyGuard(config).check(info)
                      .copy() for info in cases]
        finally:
            fast_topology._IS_MICROPYTHON = old_backend
        self.assertEqual(actual, expected)


if __name__ == "__main__":
    unittest.main()
