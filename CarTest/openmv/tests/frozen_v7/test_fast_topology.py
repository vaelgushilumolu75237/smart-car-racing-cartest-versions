"""Equivalence checks for the compact fast path topology guard."""

import sys
import unittest
import random
from pathlib import Path
from unittest.mock import patch


TEST_ROOT = Path(__file__).resolve().parents[1]
OPENMV_DIR = TEST_ROOT / "openmv"
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))

import config
import fast_topology
import fast_path
import source_selector
import surface_detector


WIDTH = config.SURFACE_GRID_WIDTH
HEIGHT = config.SURFACE_GRID_HEIGHT
FLOOR = config.SURFACE_ROI_FLOOR_Y
TOTAL = WIDTH * HEIGHT


def make_corridor_mask(center=39.5, half_width=17, y_start=20, y_end=59):
    mask = bytearray(TOTAL)
    left = center - half_width
    right = center + half_width
    for y in range(HEIGHT):
        for x in range(WIDTH):
            if y < y_start or y > y_end or x < left or x > right:
                mask[y * WIDTH + x] = 1
    return mask


def row_counts_for(mask, floor=FLOOR):
    rows = bytearray(HEIGHT)
    for y in range(floor, HEIGHT):
        offset = y * WIDTH
        count = 0
        for x in range(WIDTH):
            index = offset + x
            if mask[index]:
                count += 1
        rows[y] = count
    return rows


def make_mask_info(mask, floor=FLOOR, complete=True):
    known = bytearray(TOTAL)
    luma = bytearray([25]) * TOTAL
    road = bytearray(TOTAL)
    if complete:
        for y in range(floor, HEIGHT):
            offset = y * WIDTH
            for x in range(WIDTH):
                index = offset + x
                known[index] = 1
                if not mask[index]:
                    luma[index] = 220
                    road[index] = 255
    return {
        "valid": True,
        "reason": "OK",
        "luma": luma,
        "road_mask": road,
        "edge_mask": mask,
        "known_region": known,
        "row_counts": row_counts_for(mask, floor),
        "road_row_counts": row_counts_for(road, floor),
        "width": WIDTH,
        "height": HEIGHT,
        "floor": floor,
    }


def mark_unknown_rows(info, start_y, end_y):
    """Clear a half-open row range exactly as a non-trusted gray block does."""
    for y in range(start_y, end_y):
        start = y * WIDTH
        end = start + WIDTH
        info["known_region"][start:end] = bytes(WIDTH)
        info["edge_mask"][start:end] = bytes(WIDTH)
        if "road_mask" in info:
            info["road_mask"][start:end] = bytes(WIDTH)
        if "luma" in info:
            info["luma"][start:end] = bytes(WIDTH)
        if "road_row_counts" in info:
            info["road_row_counts"][y] = 0
    info["row_counts"] = row_counts_for(
        info["edge_mask"], info["floor"])


class FastTopologyGuardTests(unittest.TestCase):
    def setUp(self):
        self.guard = fast_topology.FastTopologyGuard(config)

    def _assert_legacy_equivalence(self, mask, floor=FLOOR):
        info = make_mask_info(mask, floor)
        actual = self.guard.check(info)
        self.assertEqual(actual["island_evidence"],
                         surface_detector._has_internal_mask_component(
                             mask, self._settings_with_floor(floor)))
        self.assertEqual(actual["branch_evidence"],
                         surface_detector._has_internal_open_band(
                             mask, self._settings_with_floor(floor),
                             info["row_counts"]))
        self.assertEqual(actual["ambiguous"],
                         actual["island_evidence"] or
                         actual["branch_evidence"])
        return actual

    @staticmethod
    def _settings_with_floor(floor):
        class Settings:
            SURFACE_GRID_WIDTH = config.SURFACE_GRID_WIDTH
            SURFACE_GRID_HEIGHT = config.SURFACE_GRID_HEIGHT
            SURFACE_ROI_FLOOR_Y = floor
            SURFACE_INTERNAL_COMPONENT_MIN_AREA = \
                config.SURFACE_INTERNAL_COMPONENT_MIN_AREA
            SURFACE_INTERNAL_COMPONENT_MIN_EXTENT = \
                config.SURFACE_INTERNAL_COMPONENT_MIN_EXTENT
            SURFACE_INTERNAL_COMPONENT_MAX_EXTENT = \
                config.SURFACE_INTERNAL_COMPONENT_MAX_EXTENT
            SURFACE_INTERNAL_COMPONENT_MAX_AREA = \
                config.SURFACE_INTERNAL_COMPONENT_MAX_AREA
            SURFACE_INTERNAL_HOLE_MIN_AREA = \
                config.SURFACE_INTERNAL_HOLE_MIN_AREA
            SURFACE_OPEN_BAND_MIN_ROWS = config.SURFACE_OPEN_BAND_MIN_ROWS
            SURFACE_OPEN_BAND_SUPPORT_ROWS = \
                config.SURFACE_OPEN_BAND_SUPPORT_ROWS
            SURFACE_OPEN_BAND_MAX_RUN_WIDTH = \
                config.SURFACE_OPEN_BAND_MAX_RUN_WIDTH
            SURFACE_OPEN_BAND_MIN_PAIR_GAP = \
                config.SURFACE_OPEN_BAND_MIN_PAIR_GAP
            SURFACE_OPEN_BAND_WIDE_ROWS = config.SURFACE_OPEN_BAND_WIDE_ROWS
        return Settings

    def test_ring_and_solid_patch_match_legacy_component_and_hole_semantics(self):
        ring = make_corridor_mask()
        for y in range(41, 50):
            for x in range(34, 43):
                radius = ((x - 38) ** 2 + (y - 45) ** 2) ** 0.5
                if 2.4 <= radius <= 3.5:
                    ring[y * WIDTH + x] = 1
        result = self._assert_legacy_equivalence(ring)
        self.assertTrue(result["island_evidence"])
        self.assertTrue(result["ambiguous"])
        self.assertEqual(result["reason"], "ISLAND")

        solid = bytearray(TOTAL)
        for y in range(40, 45):
            for x in range(34, 39):
                solid[y * WIDTH + x] = 1
        result = self._assert_legacy_equivalence(solid)
        self.assertFalse(result["island_evidence"])
        self.assertFalse(result["ambiguous"])

    def test_crop_contacts_and_diagonal_only_component_link_match_legacy(self):
        def add_ring(mask, x0, y0, size=5):
            for y in range(y0, y0 + size):
                for x in range(x0, x0 + size):
                    if (x == x0 or x == x0 + size - 1 or
                            y == y0 or y == y0 + size - 1):
                        mask[y * WIDTH + x] = 1

        crop_masks = []
        left = bytearray(TOTAL)
        add_ring(left, 0, 35)
        crop_masks.append(("left", left))
        top = bytearray(TOTAL)
        add_ring(top, 30, FLOOR)
        crop_masks.append(("roi_floor", top))
        bottom = bytearray(TOTAL)
        add_ring(bottom, 45, HEIGHT - 5)
        crop_masks.append(("bottom", bottom))
        for name, mask in crop_masks:
            with self.subTest(crop=name):
                result = self._assert_legacy_equivalence(mask)
                self.assertFalse(result["island_evidence"])

        # Two hollow components touch at exactly one diagonal pair. With an
        # area floor of 25, only the specified 8-connected union qualifies.
        diagonal = bytearray(TOTAL)
        add_ring(diagonal, 20, 30)
        add_ring(diagonal, 25, 35)
        with patch.object(config, "SURFACE_INTERNAL_COMPONENT_MIN_AREA", 25):
            settings = self._settings_with_floor(FLOOR)
            guard = fast_topology.FastTopologyGuard(config)
            info = make_mask_info(diagonal)
            result = guard.check(info)
            self.assertEqual(result["island_evidence"],
                             surface_detector._has_internal_mask_component(
                                 diagonal, settings))
            self.assertTrue(result["island_evidence"])

    def test_open_band_aliasing_crossbar_and_roi_edge_runs_match_legacy(self):
        alias_gap = make_corridor_mask()
        for y in range(32, 35):
            for x in range(WIDTH):
                alias_gap[y * WIDTH + x] = 0
        result = self._assert_legacy_equivalence(alias_gap)
        self.assertFalse(result["branch_evidence"])

        crossbar = bytearray(TOTAL)
        for y in range(28, 32):
            for x in range(WIDTH):
                crossbar[y * WIDTH + x] = 1
        for y in range(35, HEIGHT):
            crossbar[y * WIDTH + 15] = 1
            crossbar[y * WIDTH + 64] = 1
        result = self._assert_legacy_equivalence(crossbar)
        self.assertTrue(result["branch_evidence"])
        self.assertEqual(result["reason"], "OPEN_BAND")

        top_connected = bytearray(TOTAL)
        for y in range(20, 25):
            for x in range(WIDTH):
                top_connected[y * WIDTH + x] = 1
        for y in range(29, HEIGHT):
            top_connected[y * WIDTH + 15] = 1
            top_connected[y * WIDTH + 64] = 1
        self.assertFalse(self._assert_legacy_equivalence(
            top_connected)["branch_evidence"])

        bottom_connected = bytearray(TOTAL)
        for y in range(20, 25):
            bottom_connected[y * WIDTH + 15] = 1
            bottom_connected[y * WIDTH + 64] = 1
        for y in range(29, HEIGHT):
            for x in range(WIDTH):
                bottom_connected[y * WIDTH + x] = 1
        result = self._assert_legacy_equivalence(bottom_connected)
        self.assertFalse(result["branch_evidence"])

    def test_cropped_corridor_and_zebra_seams_are_not_forced_into_islands(self):
        cropped = make_corridor_mask(y_start=20, y_end=43)
        result = self._assert_legacy_equivalence(cropped)
        self.assertFalse(result["island_evidence"])

        zebra = make_corridor_mask()
        for y in (31, 38, 45, 52):
            for x in range(23, 57):
                zebra[y * WIDTH + x] = 1
        result = self._assert_legacy_equivalence(zebra)
        self.assertFalse(result["island_evidence"])
        self.assertFalse(result["branch_evidence"])

    def test_incomplete_row_is_a_segment_boundary_not_a_filled_pixel(self):
        ring = make_corridor_mask()
        for y in range(41, 50):
            for x in range(34, 43):
                radius = ((x - 38) ** 2 + (y - 45) ** 2) ** 0.5
                if 2.4 <= radius <= 3.5:
                    ring[y * WIDTH + x] = 1
        info = make_mask_info(ring, complete=True)
        info["known_region"][45 * WIDTH + 38] = 0
        info["row_counts"] = row_counts_for(info["edge_mask"], FLOOR)
        result = self.guard.check(info)
        self.assertFalse(result["island_evidence"])
        self.assertFalse(result["branch_evidence"])
        self.assertTrue(result["ambiguous"])
        self.assertEqual(result["reason"], "UNKNOWN_REGION")
        self.assertFalse(result["complete"])
        self.assertEqual(result["trusted_segment_count"], 0)

        info = make_mask_info(ring, complete=True)
        info["row_counts"][45] += 1
        result = self.guard.check(info)
        self.assertEqual(result["reason"], "INVALID_MASK_INFO")
        self.assertFalse(result["ambiguous"])

    def test_known_closed_island_is_checked_across_remote_unknown_regions(self):
        ring = self._ring_mask()
        cases = (
            ("top", 20, 32),
            ("middle", 35, 36),
            ("bottom", 55, HEIGHT),
        )
        for name, start, end in cases:
            with self.subTest(region=name):
                info = make_mask_info(ring, complete=True)
                mark_unknown_rows(info, start, end)
                result = fast_topology.FastTopologyGuard(config).check(info)
                self.assertTrue(result["island_evidence"], result)
                self.assertTrue(result["ambiguous"], result)
                self.assertEqual(result["reason"], "ISLAND")
                self.assertFalse(result["complete"])
                self.assertGreaterEqual(result["trusted_segment_count"], 1)

    def test_partial_known_road_remains_usable_but_closed_island_rejects(self):
        info = make_mask_info(make_corridor_mask(), complete=True)
        # This removes an entire threshold block above the selected reference.
        mark_unknown_rows(info, 20, 32)
        topology = fast_topology.FastTopologyGuard(config).check(info)
        self.assertEqual(topology["reason"], "PARTIAL_KNOWN_ROI")
        self.assertFalse(topology["ambiguous"])
        self.assertFalse(topology["complete"])
        path_result = fast_path.FastPathDetector(config).detect(info)
        self.assertTrue(path_result["valid"], path_result.get("reason"))
        self.assertEqual(path_result["reference_center_x"], 39.5)

        ring_info = make_mask_info(self._ring_mask(), complete=True)
        mark_unknown_rows(ring_info, 20, 32)
        ring_result = fast_path.FastPathDetector(config).detect(ring_info)
        self.assertFalse(ring_result["valid"])
        self.assertEqual(ring_result["reason"], "ISLAND")

    def test_known_open_band_is_checked_inside_partial_roi_segment(self):
        branch = bytearray(TOTAL)
        for y in range(28, 32):
            for x in range(WIDTH):
                branch[y * WIDTH + x] = 1
        for y in range(35, HEIGHT):
            branch[y * WIDTH + 15] = 1
            branch[y * WIDTH + 64] = 1
        info = make_mask_info(branch, complete=True)
        mark_unknown_rows(info, 20, 25)
        result = fast_topology.FastTopologyGuard(config).check(info)
        self.assertTrue(result["branch_evidence"], result)
        self.assertTrue(result["ambiguous"], result)
        self.assertEqual(result["reason"], "OPEN_BAND")
        self.assertFalse(result["complete"])
        self.assertEqual(result["trusted_segment_count"], 1)

    def test_single_unknown_row_adjacent_to_contour_is_not_closed_by_unknown(self):
        ring = self._ring_mask()
        info = make_mask_info(ring, complete=True)
        # The unknown row is immediately below the ring's last dark row. The
        # trusted segment ends there, so unknown pixels cannot complete a
        # contour or be counted as the hole's white boundary.
        ring_end = max(
            y for y in range(FLOOR, HEIGHT)
            if any(ring[y * WIDTH + x] for x in range(34, 43)))
        mark_unknown_rows(info, ring_end + 1, ring_end + 2)
        result = fast_topology.FastTopologyGuard(config).check(info)
        self.assertFalse(result["ambiguous"], result)
        self.assertEqual(result["reason"], "PARTIAL_KNOWN_ROI")
        self.assertEqual(result["trusted_segment_count"], 2)

    def test_partial_known_without_any_full_row_fails_unknown(self):
        info = make_mask_info(make_corridor_mask(), complete=False)
        result = self.guard.check(info)
        self.assertEqual(result["reason"], "UNKNOWN_REGION")
        self.assertFalse(result["island_evidence"])
        self.assertFalse(result["branch_evidence"])
        self.assertTrue(result["ambiguous"])
        self.assertFalse(result["complete"])
        self.assertEqual(result["trusted_segment_count"], 0)
        self.assertEqual(result["fully_known_rows"], 0)

    def test_remote_unknown_column_and_known_ring_fail_closed(self):
        cases = (
            ("clear_corridor", make_corridor_mask(), None),
            ("known_ring", self._ring_mask(), None),
            ("known_ring_one_full_row", self._ring_mask(), 40),
        )
        for name, mask, full_row in cases:
            with self.subTest(scene=name):
                info = make_mask_info(mask, complete=True)
                # A permitted partial-width threshold block can leave one
                # remote unknown cell in every ROI row. The center corridor
                # (and, in the second case, the closed ring) remains known,
                # but no complete row is available to prove topology.
                for y in range(FLOOR, HEIGHT):
                    if y != full_row:
                        index = y * WIDTH + 79
                        info["known_region"][index] = 0
                        info["edge_mask"][index] = 0
                        info["road_mask"][index] = 0
                info["row_counts"] = row_counts_for(
                    info["edge_mask"], FLOOR)
                topology = fast_topology.FastTopologyGuard(config).check(info)
                self.assertEqual(topology["reason"], "UNKNOWN_REGION")
                self.assertTrue(topology["ambiguous"])
                self.assertFalse(topology["island_evidence"])
                self.assertFalse(topology["branch_evidence"])
                self.assertEqual(topology["trusted_segment_count"], 0)
                path_result = fast_path.FastPathDetector(config).detect(info)
                self.assertFalse(path_result["valid"], path_result)
                self.assertEqual(path_result["reason"], "UNKNOWN_REGION")

    def test_unknown_row_mask_bits_are_validated_but_never_topology_inputs(self):
        info = make_mask_info(make_corridor_mask(), complete=True)
        # Simulate an edge plane carrying stale dark bytes through one unknown
        # row. The row checksum remains the established all-mask count, while
        # the incomplete row is excluded from every topology kernel.
        start = 40 * WIDTH
        info["known_region"][start:start + WIDTH] = bytes(WIDTH)
        info["row_counts"] = row_counts_for(info["edge_mask"], FLOOR)
        result = self.guard.check(info)
        self.assertEqual(result["reason"], "PARTIAL_KNOWN_ROI")
        self.assertFalse(result["ambiguous"], result)
        self.assertEqual(result["trusted_segment_count"], 2)

    def test_result_and_scratch_buffers_are_reused_and_cleared(self):
        visited_id = id(self.guard._visited)
        queue_id = id(self.guard._queue)
        result = self._assert_legacy_equivalence(self._ring_mask())
        self.assertTrue(result["island_evidence"])

        clean = make_corridor_mask()
        info = make_mask_info(clean)
        next_result = self.guard.check(info)
        self.assertIs(result, next_result)
        self.assertFalse(next_result["ambiguous"])
        self.assertEqual(id(self.guard._visited), visited_id)
        self.assertEqual(id(self.guard._queue), queue_id)

    def test_scanline_island_fill_matches_pixel_bfs_on_seeded_masks(self):
        rng = random.Random(0x7A5C)
        for case_index in range(48):
            mask = bytearray(TOTAL)
            # Keep samples inside the ROI; vary density and shape enough to
            # cover thin runs, diagonal 8-connectivity, compact blobs and
            # disconnected components without adding scene-specific rules.
            x0 = rng.randrange(8, 40)
            x1 = rng.randrange(max(x0 + 3, 40), 72)
            y0 = rng.randrange(FLOOR + 1, 42)
            y1 = rng.randrange(max(y0 + 3, 43), HEIGHT - 1)
            density = (0.08, 0.22, 0.48, 0.72)[case_index % 4]
            for y in range(y0, y1):
                for x in range(x0, x1):
                    if rng.random() < density:
                        mask[y * WIDTH + x] = 1
            info = make_mask_info(mask)
            self.guard._current_mask = info["edge_mask"]
            self.guard._current_known = info["known_region"]
            self.guard._current_rows = info["row_counts"]
            self.guard._current_floor = FLOOR
            scanline = self.guard._internal_component_runs_host(FLOOR)
            pixel_bfs = self.guard._internal_component_host(FLOOR)
            self.assertEqual(scanline, pixel_bfs, "mask case %d" % case_index)

    @staticmethod
    def _ring_mask():
        mask = make_corridor_mask()
        for y in range(41, 50):
            for x in range(34, 43):
                radius = ((x - 38) ** 2 + (y - 45) ** 2) ** 0.5
                if 2.4 <= radius <= 3.5:
                    mask[y * WIDTH + x] = 1
        return mask

    def test_bad_dimensions_or_buffer_lengths_fail_closed(self):
        info = make_mask_info(make_corridor_mask())
        info["width"] = WIDTH - 1
        result = self.guard.check(info)
        self.assertEqual(result["reason"], "INVALID_MASK_INFO")
        self.assertFalse(result["ambiguous"])

        info = make_mask_info(make_corridor_mask())
        info["edge_mask"] = bytearray(TOTAL - 1)
        self.assertEqual(self.guard.check(info)["reason"],
                         "INVALID_MASK_INFO")

        info = make_mask_info(make_corridor_mask())
        info["floor"] = True
        self.assertEqual(self.guard.check(info)["reason"],
                         "INVALID_MASK_INFO")

    def test_projected_seam_and_shadow_fixtures_match_legacy_semantics(self):
        from tests.v5_projected_scenes import generate_scene

        scenes = (
            ("practice_straight", generate_scene(
                "topology_practice_straight_seams",
                road_type="straight", camera_down_deg=20,
                track_style="practice", camera_route_chainage_m=3.0,
                seams=True, shadow=True)),
            ("formal_straight", generate_scene(
                "topology_formal_straight_seams",
                road_type="straight", camera_down_deg=20,
                track_style="formal", camera_route_chainage_m=3.0,
                seams=True, shadow=True)),
            ("practice_curve", generate_scene(
                "topology_practice_curve_seams",
                road_type="curve", camera_down_deg=22,
                track_style="practice", camera_route_chainage_m=3.0,
                curve_radius_m=0.8, turn="left", seams=True,
                shadow=True)),
        )
        for style, scene in scenes:
            with self.subTest(scene=scene.scene_id):
                mode = ("blue_white" if style.startswith("practice")
                        else "black_white")
                image = (scene.rgb_image if mode == "blue_white"
                         else scene.gray_image)
                mask, _details = surface_detector.build_mask(
                    image, mode, config)
                result = self._assert_legacy_equivalence(mask)
                self.assertFalse(result["island_evidence"])

    def test_topology_evidence_preserves_selector_fallback_rejection(self):
        ring_result = self._assert_legacy_equivalence(self._ring_mask())
        surface = {"valid": False, "reason": "NO_CLOSED_SECTION"}
        surface.update(ring_result)
        selected = source_selector.select_candidate(
            surface, {"valid": True, "quality": 86,
                      "lateral_error": 0, "reference_center_x": 39.5})
        self.assertFalse(selected["valid"])
        self.assertEqual(selected["reason"], "SURFACE_TOPOLOGY_AMBIGUOUS")
        self.assertTrue(selected["comparison"]["island_evidence"])

    def test_default_config_limits_are_preserved_and_host_is_not_viper(self):
        self.assertEqual(self.guard.component_min_area, 12)
        self.assertEqual(self.guard.component_min_extent, 3)
        self.assertEqual(self.guard.component_max_extent, 18)
        self.assertEqual(self.guard.component_max_area, 180)
        self.assertEqual(self.guard.hole_min_area, 4)
        self.assertEqual(self.guard.open_min_rows, 3)
        self.assertEqual(self.guard.open_support_rows, 5)
        self.assertEqual(self.guard.open_max_run_width, 36)
        self.assertEqual(self.guard.open_min_pair_gap, 8)
        self.assertEqual(self.guard.open_wide_rows, 4)
        if sys.implementation.name == "cpython":
            self.assertEqual(fast_topology.COMPILER_BACKEND,
                             "cpython-shim")


if __name__ == "__main__":
    unittest.main(verbosity=2)
