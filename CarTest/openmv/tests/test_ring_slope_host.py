"""Host evidence for current-frame ring path and slope-hint gating.

All image masks and frame sequences in this file are synthetic. They verify
bounded algorithm behavior only; they are not camera or vehicle results.
"""

import sys
import types
import unittest
from array import array
from pathlib import Path


OPENMV_DIR = Path(__file__).resolve().parents[1]
if str(OPENMV_DIR) not in sys.path:
    sys.path.insert(0, str(OPENMV_DIR))
sys.modules.setdefault("pyb", types.ModuleType("pyb"))
sys.modules.setdefault("sensor", types.ModuleType("sensor"))

import config
import fast_ring_slope as ring_slope


WIDTH = 80
HEIGHT = 60
FLOOR = 8


def _ring_fixture(side=ring_slope.RING_RIGHT, port=True,
                  close_dark=False, unknown_neighbor=False,
                  crop_side_edge=None):
    road = bytearray(WIDTH * HEIGHT)
    edge = bytearray(WIDTH * HEIGHT)
    known = bytearray(WIDTH * HEIGHT)
    for y in range(FLOOR, HEIGHT):
        row = y * WIDTH
        for x in range(WIDTH):
            known[row + x] = 1

    packed = array("i")
    for y in range(18, 59):
        offset = y - 38
        if side == ring_slope.RING_RIGHT:
            inner = int(42 + (offset * offset) / 30.0 + 0.5)
            outer = inner - 24
            input_left = outer + 1
            input_right = inner - 1
            edge[y * WIDTH + outer] = 1
            edge[y * WIDTH + inner] = 1
            for x in range(outer + 1, inner):
                road[y * WIDTH + x] = 1
        else:
            inner = int(38 - (offset * offset) / 30.0 + 0.5)
            outer = inner + 24
            input_left = inner + 1
            input_right = outer - 1
            edge[y * WIDTH + inner] = 1
            edge[y * WIDTH + outer] = 1
            for x in range(inner + 1, outer):
                road[y * WIDTH + x] = 1
        if y % 2 == 0:
            packed.extend((y, input_left, input_right, 7))

    # The grayscale road mask has a pixel for every grid y even though the
    # boundary validator samples every other row in this fixture.
    for y in range(18, 59):
        offset = y - 38
        if side == ring_slope.RING_RIGHT:
            inner = int(42 + (offset * offset) / 30.0 + 0.5)
            outer = inner - 24
            road_start, road_end = outer + 1, inner
        else:
            inner = int(38 - (offset * offset) / 30.0 + 0.5)
            outer = inner + 24
            road_start, road_end = inner + 1, outer
        for x in range(road_start, road_end):
            road[y * WIDTH + x] = 1

    first_inner = int(42 + ((18 - 38) ** 2) / 30.0 + 0.5) \
        if side == ring_slope.RING_RIGHT else \
        int(38 - ((18 - 38) ** 2) / 30.0 + 0.5)
    if crop_side_edge is None:
        crop_side_edge = WIDTH - 1 if side == ring_slope.RING_RIGHT else 0
    if side == ring_slope.RING_RIGHT:
        low = first_inner - 24 + 1
        high = crop_side_edge - 1 if port else first_inner - 1
        for y in range(14, 18):
            edge[y * WIDTH + low - 1] = 1
        if not port:
            for y in range(14, 18):
                edge[y * WIDTH + high + 1] = 1
    else:
        low = crop_side_edge + 1 if port else first_inner + 1
        high = first_inner + 24 - 1
        for y in range(14, 18):
            edge[y * WIDTH + high + 1] = 1
        if not port:
            for y in range(14, 18):
                edge[y * WIDTH + low - 1] = 1
    for y in range(14, 18):
        for x in range(low, high + 1):
            road[y * WIDTH + x] = 1
        edge[y * WIDTH + crop_side_edge] = 1
        edge[y * WIDTH + (WIDTH - 1 if side == ring_slope.RING_RIGHT
                          else 0)] = 1

    if close_dark:
        for y in range(14, 18):
            edge[y * WIDTH + (76 if side == ring_slope.RING_RIGHT else 3)] = 1
    if unknown_neighbor:
        for y in range(14, 18):
            known[y * WIDTH + (76 if side == ring_slope.RING_RIGHT else 3)] = 0

    result = {
        "valid": True,
        "selected_row_ints": packed,
        "selected_row_count": len(packed) // 4,
        "selected_row_stride": 4,
        "topology_evidence": {},
    }
    support = bytearray([1] * (len(packed) // 4))
    mask_info = {"road_mask": road, "edge_mask": edge,
                 "known_region": known}
    return mask_info, result, support


def _upper_arc_port_fixture(side=ring_slope.RING_RIGHT,
                            separated_ports=False):
    """Synthetic current road whose side opening appears beside the far arc."""
    road = bytearray(WIDTH * HEIGHT)
    edge = bytearray(WIDTH * HEIGHT)
    known = bytearray(WIDTH * HEIGHT)
    for y in range(FLOOR, HEIGHT):
        row = y * WIDTH
        known[row:row + WIDTH] = bytearray([1] * WIDTH)

    packed = array("i")
    for y in range(18, 59):
        if side == ring_slope.RING_RIGHT:
            inner = (79 if y <= 20 else
                     (79 - 2 * (y - 20) if y <= 38 else
                      43 + (y - 38)))
            outer = inner - 24
            input_left = outer + 1
            input_right = inner - 1
        else:
            inner = (0 if y <= 20 else
                     (2 * (y - 20) if y <= 38 else
                      36 - (y - 38)))
            outer = inner + 24
            input_left = inner + 1
            input_right = outer - 1
        if y % 2 == 0:
            packed.extend((y, input_left, input_right, 7))
        edge[y * WIDTH + inner] = 1
        edge[y * WIDTH + outer] = 1
        for x in range(outer + 1, inner) if side == ring_slope.RING_RIGHT \
                else range(inner + 1, outer):
            road[y * WIDTH + x] = 1
        edge[y * WIDTH + (WIDTH - 1 if side == ring_slope.RING_RIGHT
                          else 0)] = 1

    # The former fixed band is a closed white road. It cannot reach the crop
    # side; the real opening first appears alongside the measured far arc.
    old_low, old_high = ((31, 54) if side == ring_slope.RING_RIGHT else
                         (25, 49))
    for y in range(14, 18):
        for x in range(old_low, old_high + 1):
            road[y * WIDTH + x] = 1
        edge[y * WIDTH + old_low - 1] = 1
        edge[y * WIDTH + old_high + 1] = 1
        edge[y * WIDTH + (WIDTH - 1 if side == ring_slope.RING_RIGHT
                          else 0)] = 1

    if separated_ports:
        # Two connected inset-port rows at y14 and y18 with a known-road
        # corridor one cell inward. The empty target rows between them must
        # remain two distinct side-port segments.
        for y in range(14, 18):
            for x in range(55, 75):
                road[y * WIDTH + x] = 1
        road[14 * WIDTH + 75] = 1
        road[14 * WIDTH + 76] = 1

    result = {
        "valid": True,
        "selected_row_ints": packed,
        "selected_row_count": len(packed) // 4,
        "selected_row_stride": 4,
        "topology_evidence": {},
    }
    support = bytearray([1] * (len(packed) // 4))
    return ({"road_mask": road, "edge_mask": edge,
             "known_region": known}, result, support)


def _near_bilateral_rows(side, measured_boundary_x, supported=True):
    packed = array("i")
    for y in (48, 49, 50):
        if side == ring_slope.RING_RIGHT:
            left = 10
            right = measured_boundary_x - 1
        else:
            left = measured_boundary_x + 1
            right = 70
        packed.extend((y, left, right, 7))
    result = {"valid": True, "selected_row_ints": packed,
              "selected_row_count": 3, "selected_row_stride": 4,
              "topology_evidence": {}}
    support = bytearray([1 if supported else 0] * 3)
    return result, support


def _outer_outlier_profile(spike_y=None, sustained_from=None,
                           second_spike_y=None):
    packed = array("i")
    for y in range(18, 59):
        inner = 10 + int(35.0 - ((y - 38) ** 2) / 40.0 + 0.5)
        outer = 50 + y // 4
        if spike_y == y or second_spike_y == y or (
                sustained_from is not None and y >= sustained_from):
            outer += 4
        packed.extend((y, inner + 1, outer - 1, 7))
    support = bytearray([1] * (len(packed) // 4))
    result = {"valid": True, "selected_row_ints": packed,
              "selected_row_count": len(packed) // 4,
              "selected_row_stride": 4, "topology_evidence": {}}
    return result, support


def _profile_result(width_slope=0.8, width_intercept=18.0,
                    center_slope=0.0, support_count=None):
    packed = array("i")
    for y in range(30, 51, 2):
        width = width_slope * y + width_intercept
        center = 40.0 + center_slope * (y - 40)
        left = int(center - width * 0.5 + 0.5)
        right = int(center + width * 0.5 + 0.5)
        packed.extend((y, left, right, 7))
    count = len(packed) // 4
    support = bytearray([1] * count)
    if support_count is not None:
        for index in range(support_count, count):
            support[index] = 0
    return ({"valid": True, "selected_row_ints": packed,
             "selected_row_count": count, "selected_row_stride": 4},
            support, count)


class FastRingSlopeTests(unittest.TestCase):
    def setUp(self):
        self.detector = ring_slope.FastRingSlope(config)

    def test_right_arc_and_connected_white_port_are_current_geometry(self):
        masks, result, support = _ring_fixture(ring_slope.RING_RIGHT)
        candidate = self.detector.detect_ring_candidate(
            masks, result, support, len(support))
        self.assertTrue(candidate["valid"], candidate)
        self.assertEqual(candidate["side_id"], ring_slope.RING_RIGHT)
        self.assertGreaterEqual(candidate["arc_rows"], 10)
        self.assertGreaterEqual(candidate["intrusion"], 6)
        self.assertEqual(candidate["clip_endpoint_x"], 79)
        self.assertGreaterEqual(candidate["port_x"], 75)
        self.assertLess(candidate["port_x"], candidate["clip_endpoint_x"])
        self.assertGreater(candidate["path_row_count"], 0)
        for y in range(HEIGHT):
            if candidate["path_valid_rows"][y]:
                x = candidate["path_x_rows"][y]
                index = y * WIDTH + x
                self.assertTrue(self.detector._visited[index])
                self.assertTrue(masks["known_region"][index])
                self.assertTrue(masks["road_mask"][index])

    def test_left_arc_is_mirrored_and_connected_to_real_crop_side_cell(self):
        masks, result, support = _ring_fixture(ring_slope.RING_LEFT)
        candidate = self.detector.detect_ring_candidate(
            masks, result, support, len(support))
        self.assertTrue(candidate["valid"], candidate)
        self.assertEqual(candidate["side_id"], ring_slope.RING_LEFT)
        self.assertEqual(candidate["clip_endpoint_x"], 0)
        self.assertLessEqual(candidate["port_x"], 4)
        self.assertGreaterEqual(candidate["port_x"], 0)

    def test_five_percent_inset_accepts_nearest_safe_crop_side_cell(self):
        cases = (
            (ring_slope.RING_LEFT, 0, 3),
            (ring_slope.RING_LEFT, 1, 4),
            (ring_slope.RING_RIGHT, WIDTH - 1, 76),
            (ring_slope.RING_RIGHT, WIDTH - 2, 75),
        )
        for side, crop_side_edge, expected_x in cases:
            masks, result, support = _ring_fixture(
                side, crop_side_edge=crop_side_edge)
            detector = ring_slope.FastRingSlope(config)
            candidate = detector.detect_ring_candidate(
                masks, result, support, len(support))
            self.assertTrue(candidate["valid"], (side, crop_side_edge,
                                                  candidate["reason"]))
            self.assertEqual(candidate["port_x"], expected_x)
            self.assertEqual(candidate["port_inset_margin_cells"], 4)
            self.assertEqual(candidate["clip_endpoint_x"],
                             0 if side == ring_slope.RING_LEFT else WIDTH - 1)
            self.assertEqual(candidate["port_inset_margin_cells"], 4)
            index = candidate["port_y"] * WIDTH + candidate["port_x"]
            self.assertTrue(masks["known_region"][index])
            self.assertTrue(masks["road_mask"][index])
            self.assertTrue(detector._dark_clear(
                candidate["port_x"], candidate["port_y"],
                masks["edge_mask"], masks["known_region"]))

    def test_far_arc_clearance_port_is_found_beyond_old_upstream_band(self):
        for side in (ring_slope.RING_RIGHT, ring_slope.RING_LEFT):
            masks, result, support = _upper_arc_port_fixture(side)
            detector = ring_slope.FastRingSlope(config)
            candidate = detector.detect_ring_candidate(
                masks, result, support, len(support))
            self.assertTrue(candidate["valid"], (side, candidate))
            self.assertEqual(candidate["port_search_y_min"], 14)
            self.assertGreater(candidate["port_search_y_max"], 17)
            self.assertEqual(candidate["port_y"], 18)
            if side == ring_slope.RING_RIGHT:
                self.assertEqual(candidate["port_x"], 76)
            else:
                self.assertEqual(candidate["port_x"], 3)

    def test_two_separated_reachable_side_ports_are_ambiguous(self):
        masks, result, support = _upper_arc_port_fixture(
            ring_slope.RING_RIGHT, separated_ports=True)
        candidate = ring_slope.FastRingSlope(config).detect_ring_candidate(
            masks, result, support, len(support))
        self.assertFalse(candidate["valid"])
        self.assertEqual(candidate["reason"], "MULTIPLE_PORTS")

    def test_one_isolated_outer_measurement_is_logged_but_sustained_jump_rejects(self):
        detector = ring_slope.FastRingSlope(config)
        result, support = _outer_outlier_profile(spike_y=39)
        self.assertTrue(detector._extract_supported_rows(
            result, support, len(support)))
        arc = detector._arc_candidate(ring_slope.RING_LEFT)
        self.assertIsNotNone(arc)
        self.assertEqual(arc["outer_outlier_excluded_y"], 39)
        self.assertGreaterEqual(arc["rows"] - 1, 10)
        self.assertGreaterEqual(arc["y_max"] - arc["y_min"], 12)

        for result, support in (
                _outer_outlier_profile(sustained_from=39),
                _outer_outlier_profile(spike_y=29, second_spike_y=39)):
            detector = ring_slope.FastRingSlope(config)
            self.assertTrue(detector._extract_supported_rows(
                result, support, len(support)))
            self.assertIsNone(detector._arc_candidate(ring_slope.RING_LEFT))

    def test_dark_close_unknown_and_no_port_are_rejected(self):
        cases = ((False, False, False), (True, False, True),
                 (False, True, True))
        for side in (ring_slope.RING_RIGHT, ring_slope.RING_LEFT):
            for close_dark, unknown, expect_reject in cases:
                masks, result, support = _ring_fixture(
                    side, port=True, close_dark=close_dark,
                    unknown_neighbor=unknown)
                candidate = ring_slope.FastRingSlope(config).detect_ring_candidate(
                    masks, result, support, len(support))
                self.assertEqual(not candidate["valid"], expect_reject,
                                 (side, close_dark, unknown,
                                  candidate["reason"]))
        masks, result, support = _ring_fixture(port=False)
        candidate = ring_slope.FastRingSlope(config).detect_ring_candidate(
            masks, result, support, len(support))
        self.assertFalse(candidate["valid"])
        self.assertEqual(candidate["reason"], "NO_CONNECTED_PORT")

    def test_border_pixels_alone_without_measured_arc_cannot_make_ring(self):
        masks, _result, _support = _ring_fixture(ring_slope.RING_LEFT)
        candidate = ring_slope.FastRingSlope(config).detect_ring_candidate(
            masks, {"valid": True, "selected_row_ints": array("i"),
                    "selected_row_count": 0, "selected_row_stride": 4,
                    "topology_evidence": {}}, bytearray(), 0)
        self.assertFalse(candidate["valid"])
        self.assertEqual(candidate["reason"], "NO_SUPPORTED_BILATERAL_ROWS")

    def test_nearest_safe_supported_seed_fallback_is_mirrored(self):
        for side in (ring_slope.RING_RIGHT, ring_slope.RING_LEFT):
            masks, result, support = _ring_fixture(side)
            last = result["selected_row_count"] - 1
            last_base = last * 4
            seed_y = result["selected_row_ints"][last_base]
            seed_x = (result["selected_row_ints"][last_base + 1] +
                      result["selected_row_ints"][last_base + 2]) // 2
            # The nearest supported row has an unsafe known-dark neighbor at
            # y+1; the next supported row remains inside the four-row search.
            masks["edge_mask"][(seed_y + 1) * WIDTH + seed_x] = 1
            detector = ring_slope.FastRingSlope(config)
            candidate = detector.detect_ring_candidate(
                masks, result, support, len(support))
            self.assertTrue(candidate["valid"], (side, candidate))
            fallback_base = (last - 1) * 4
            fallback_x = (result["selected_row_ints"][fallback_base + 1] +
                          result["selected_row_ints"][fallback_base + 2]) // 2
            self.assertNotEqual(candidate["near_seed_x"], seed_x)
            self.assertEqual(candidate["near_seed_x"], fallback_x)
            self.assertEqual(candidate["near_seed_y"], seed_y - 2)
            self.assertEqual(candidate["near_center_x"], float(fallback_x))
            self.assertEqual(candidate["near_center_y"], seed_y - 2)

    def test_all_near_seed_candidates_without_clearance_are_rejected(self):
        masks, result, support = _ring_fixture(ring_slope.RING_RIGHT)
        # The last three supported center rows lie at 58, 56, and 54. A
        # known-dark line at 56 is within two cells of each candidate seed.
        edge_row = 56 * WIDTH
        masks["edge_mask"][edge_row:edge_row + WIDTH] = bytearray(
            [1] * WIDTH)
        candidate = ring_slope.FastRingSlope(config).detect_ring_candidate(
            masks, result, support, len(support))
        self.assertFalse(candidate["valid"])
        self.assertEqual(candidate["reason"], "NO_SEED_BOUNDARY_CLEARANCE")

        masks, result, support = _ring_fixture(ring_slope.RING_RIGHT)
        for packed_index in range(result["selected_row_count"] - 3,
                                  result["selected_row_count"]):
            base = packed_index * 4
            y = result["selected_row_ints"][base]
            x = (result["selected_row_ints"][base + 1] +
                 result["selected_row_ints"][base + 2]) // 2
            masks["known_region"][y * WIDTH + x] = 0
        candidate = ring_slope.FastRingSlope(config).detect_ring_candidate(
            masks, result, support, len(support))
        self.assertFalse(candidate["valid"])
        self.assertEqual(candidate["reason"], "NO_NEAR_ROAD_SEED")

    def test_unsupported_rows_and_topology_branch_cannot_seed_ring(self):
        masks, result, support = _ring_fixture(ring_slope.RING_RIGHT)
        unsupported = bytearray(len(support))
        candidate = self.detector.detect_ring_candidate(
            masks, result, unsupported, len(unsupported))
        self.assertFalse(candidate["valid"])
        self.assertEqual(candidate["reason"], "NO_SUPPORTED_BILATERAL_ROWS")

        masks, result, support = _ring_fixture(ring_slope.RING_RIGHT)
        result["topology_evidence"] = {"branch_evidence": True}
        candidate = ring_slope.FastRingSlope(config).detect_ring_candidate(
            masks, result, support, len(support))
        self.assertFalse(candidate["valid"])
        self.assertEqual(candidate["reason"], "TOPOLOGY_BRANCH_OR_AMBIGUITY")

    def test_viper_kernel_matches_host_reference_on_same_masks(self):
        masks, result, support = _ring_fixture(ring_slope.RING_RIGHT)
        detector = ring_slope.FastRingSlope(config)
        candidate = detector.detect_ring_candidate(
            masks, result, support, len(support))
        self.assertTrue(candidate["valid"], candidate)
        arc = detector._arc_candidate(ring_slope.RING_RIGHT)
        detector._flood_port(masks, arc)
        start = detector._port_start_y
        end = detector._port_end_y
        seed_y = detector._candidate["near_seed_y"]
        seed_x = detector._candidate["near_seed_x"]
        seed = seed_y * WIDTH + seed_x
        host_tail = detector._flood_component_host(
            seed, start, end, ring_slope.RING_RIGHT)
        host_visited = bytes(detector._visited)
        host_ports = bytes(detector._port_rows)
        host_parent = array("i", detector._parent)
        viper_tail = ring_slope._flood_component_viper(
            detector, seed, start, end, ring_slope.RING_RIGHT)
        self.assertEqual(viper_tail, host_tail)
        self.assertEqual(bytes(detector._visited), host_visited)
        self.assertEqual(bytes(detector._port_rows), host_ports)
        self.assertEqual(detector._parent, host_parent)

    def test_three_fresh_keep_frames_enter_and_partial_edge_holds_inside(self):
        detector = ring_slope.FastRingSlope(config)
        candidate = detector._candidate
        candidate.update({"valid": True, "side_id": ring_slope.RING_RIGHT,
                          "port_y": 8})
        for expected in (1, 2, 3):
            state = detector.update_ring_state(True, 4, True, False)
            self.assertEqual(state["pending_count"], expected)
        self.assertEqual(state["state"], "ENTRY_RIGHT")
        candidate.update({"valid": False, "side_id": ring_slope.RING_NONE,
                          "partial_inner_current": True})
        state = detector.update_ring_state(True, 4, True, False)
        self.assertEqual(state["state"], "INSIDE_RIGHT")
        self.assertTrue(state["active"])
        self.assertEqual(state["reason"], "CURRENT_PARTIAL_INNER_EDGE")

    def test_unknown_edge_does_not_exit_and_current_exit_needs_three_frames(self):
        detector = ring_slope.FastRingSlope(config)
        candidate = detector._candidate
        candidate.update({"valid": True, "side_id": ring_slope.RING_RIGHT,
                          "port_y": 8})
        for _ in range(3):
            state = detector.update_ring_state(True, 1, True, False)
        candidate["partial_inner_current"] = True
        state = detector.update_ring_state(True, 1, True, False)
        self.assertEqual(state["state"], "INSIDE_RIGHT")
        candidate.update({"valid": False, "side_id": ring_slope.RING_NONE,
                          "partial_inner_current": False,
                          "inner_edge_receding": False,
                          "inner_edge_near_clip": False,
                          "inner_edge_disappeared": False})
        state = detector.update_ring_state(True, 1, True, True)
        self.assertEqual(state["state"], "INSIDE_RIGHT")
        self.assertEqual(state["exit_count"], 0)
        candidate["inner_edge_disappeared"] = True
        candidate["inner_edge_receding"] = True
        candidate["inner_edge_near_clip"] = True
        for expected in (1, 2, 3):
            state = detector.update_ring_state(True, 1, True, True)
            self.assertEqual(state["exit_count"], expected)
        self.assertEqual(state["state"], "EXIT")
        for _ in range(3):
            state = detector.update_ring_state(True, 1, True, True)
        self.assertEqual(state["state"], "IDLE")

    def test_detected_near_bilateral_exit_is_directional_mirrored_and_consecutive(self):
        cases = (
            (ring_slope.RING_RIGHT, 60, 40, 79),
            (ring_slope.RING_LEFT, 20, 40, 0),
        )
        for side, anchor, inward_x, outward_x in cases:
            detector = ring_slope.FastRingSlope(config)
            candidate = detector._candidate
            candidate.update({"valid": True, "side_id": side,
                              "port_y": 18, "near_inner_x": anchor})
            for _ in range(3):
                state = detector.update_ring_state(True, 8, True, False)
            candidate.update({"valid": False, "side_id": ring_slope.RING_NONE,
                              "partial_inner_current": True})
            state = detector.update_ring_state(True, 8, True, False)
            self.assertEqual(state["state"],
                             "INSIDE_RIGHT" if side == ring_slope.RING_RIGHT
                             else "INSIDE_LEFT")
            detector._last_inner_x = anchor

            # An equally large movement toward the lane interior must remain
            # unknown and cannot accumulate exit confirmation.
            result, support = _near_bilateral_rows(side, inward_x)
            observed = detector.detect_ring_candidate(
                {}, result, support, len(support))
            expected_measured = inward_x
            self.assertEqual(observed["near_inner_x"], expected_measured)
            self.assertFalse(observed["inner_edge_receding"])
            self.assertFalse(observed["inner_edge_disappeared"])
            state = detector.update_ring_state(True, 8, True, True)
            self.assertEqual(state["exit_count"], 0)

            result, support = _near_bilateral_rows(side, outward_x)
            observed = detector.detect_ring_candidate(
                {}, result, support, len(support))
            self.assertTrue(observed["inner_edge_receding"])
            self.assertTrue(observed["inner_edge_near_clip"])
            self.assertTrue(observed["inner_edge_disappeared"])
            self.assertEqual(observed["inner_edge_rows"], 3)
            state = detector.update_ring_state(True, 8, True, False)
            self.assertEqual(state["exit_count"], 0)

            # A missing current bilateral observation breaks the sequence.
            result, support = _near_bilateral_rows(
                side, outward_x, supported=False)
            observed = detector.detect_ring_candidate(
                {}, result, support, len(support))
            self.assertFalse(observed["inner_edge_disappeared"])
            state = detector.update_ring_state(True, 8, True, True)
            self.assertEqual(state["exit_count"], 0)

            result, support = _near_bilateral_rows(side, outward_x)
            for expected in (1, 2, 3):
                observed = detector.detect_ring_candidate(
                    {}, result, support, len(support))
                self.assertTrue(observed["inner_edge_disappeared"])
                state = detector.update_ring_state(True, 8, True, True)
                self.assertEqual(state["exit_count"], expected)
            self.assertEqual(state["state"], "EXIT")

    def test_keep_gap_and_ambiguous_candidate_clear_latched_state(self):
        detector = ring_slope.FastRingSlope(config)
        candidate = detector._candidate
        candidate.update({"valid": True, "side_id": ring_slope.RING_LEFT,
                          "port_y": 8})
        for _ in range(3):
            state = detector.update_ring_state(True, 7, True, False)
        self.assertEqual(state["state"], "ENTRY_LEFT")
        state = detector.update_ring_state(False, 7, True, False)
        self.assertEqual(state["state"], "IDLE")
        candidate.update({"valid": True, "side_id": ring_slope.RING_RIGHT,
                          "port_y": 8})
        for _ in range(3):
            state = detector.update_ring_state(True, 7, True, False)
        candidate["ambiguous"] = True
        state = detector.update_ring_state(True, 7, True, True)
        self.assertEqual(state["state"], "IDLE")
        self.assertEqual(state["reason"], "RING_SIDE_AMBIGUOUS")

    def test_candidate_without_fresh_keep_stays_preview_and_does_not_latch(self):
        detector = ring_slope.FastRingSlope(config)
        detector._candidate.update({
            "valid": True, "side_id": ring_slope.RING_RIGHT, "port_y": 8})
        state = detector.update_ring_state(False, 3, True, False)
        self.assertEqual(state["state"], "PREVIEW")
        self.assertFalse(state["active"])
        self.assertEqual(state["pending_count"], 0)

    def test_slope_learns_ten_frames_then_requires_three_candidates(self):
        detector = ring_slope.FastRingSlope(config)
        base, base_support, base_count = _profile_result(0.8, 18.1)
        for _ in range(10):
            profile = detector.update_slope(
                base, base_support, base_count, True, True, "IDLE", {})
        self.assertEqual(profile["stage"], "BASELINE")
        baseline = profile["baseline_zero_width_y"]
        candidate, candidate_support, candidate_count = _profile_result(
            1.8, -27.0)
        for expected in (1, 2, 3):
            profile = detector.update_slope(
                candidate, candidate_support, candidate_count,
                True, True, "IDLE", {})
            self.assertEqual(profile["candidate_count"], expected)
        self.assertEqual(profile["hint"], 1)
        self.assertGreaterEqual(
            profile["zero_width_y"] - baseline, 6.0)
        self.assertGreaterEqual(profile["width_slope"] /
                                profile["baseline_width_slope"], 1.35)

    def test_slope_excludes_curves_ring_single_missing_rows_and_no_baseline(self):
        detector = ring_slope.FastRingSlope(config)
        candidate, support, count = _profile_result(1.8, -27.0)
        for _ in range(4):
            profile = detector.update_slope(
                candidate, support, count, True, True, "IDLE", {})
        self.assertEqual(profile["hint"], 0)
        self.assertEqual(profile["stage"], "LEARNING_BASELINE")

        baseline, base_support, base_count = _profile_result(0.8, 18.1)
        for _ in range(10):
            detector.update_slope(
                baseline, base_support, base_count, True, True, "IDLE", {})
        for _ in range(3):
            profile = detector.update_slope(
                candidate, support, count, True, True, "IDLE", {})
        self.assertEqual(profile["hint"], 1)

        curved, curved_support, curved_count = _profile_result(
            1.8, -27.0, center_slope=0.20)
        profile = detector.update_slope(
            curved, curved_support, curved_count, True, True, "IDLE", {})
        self.assertEqual(profile["hint"], 0)
        profile = detector.update_slope(
            candidate, support, count, True, True, "PREVIEW", {})
        self.assertEqual(profile["hint"], 0)
        profile = detector.update_slope(
            candidate, support, count, True, True, "IDLE", {})
        self.assertEqual(profile["hint"], 0)
        profile = detector.update_slope(
            candidate, support, count, True, True, "IDLE",
            {"branch_evidence": True})
        self.assertEqual(profile["hint"], 0)
        profile = detector.update_slope(
            candidate, support, count, False, True, "IDLE", {})
        self.assertEqual(profile["hint"], 0)
        self.assertEqual(profile["reason"], "CURRENT_ROUTE_INVALID")

        detector.clear("TEST_RESET")
        shallow, shallow_support, shallow_count = _profile_result(0.05, 35.0)
        profile = detector.update_slope(
            shallow, shallow_support, shallow_count, True, True, "IDLE", {})
        self.assertEqual(profile["stage"], "NEED_BASELINE")


if __name__ == "__main__":
    unittest.main(verbosity=2)
