"""Private 45 cm straight-track row-width calibration for the ring observer.

Widths are measured in the 80x60 detector grid.  The recorded 450 mm value
describes the physical straight used to collect the data; it is not used to
convert image coordinates into metric geometry.

The runtime must call :meth:`accept_rows` only for fresh, accepted bilateral
road rows while the car is stationary on a topology-clear straight.  This
module checks row geometry and temporal stability as a second fail-closed
gate.  It never reads an image or the ordinary single-edge cache.
"""

try:
    import ujson as _json
except ImportError:  # CPython host tests
    import json as _json

try:
    import binascii as _binascii
except ImportError:  # pragma: no cover - OpenMV and CPython provide this
    _binascii = None

from array import array
import os


WIDTH = 80
HEIGHT = 60
FLOOR_Y = 8
REFERENCE_Y = 40
SENSOR_ID = 0x1324
CROP_X = 0
CROP_Y = 60
CROP_WIDTH = 320
CROP_HEIGHT = 120
REFERENCE_WIDTH_MM = 450
FORMAT_VERSION = 2
RUNTIME_API_VERSION = 1
REQUIRED_FRAMES = 64
MAX_ROW_SAMPLES = 64
MAX_ATTEMPTS = 600
MIN_ROW_SAMPLES = 32


def _is_int(value):
    return isinstance(value, int) and not isinstance(value, bool)


def _u32_crc(data):
    if _binascii is not None and hasattr(_binascii, "crc32"):
        return int(_binascii.crc32(data)) & 0xffffffff
    crc = 0xffffffff
    for value in data:
        crc ^= value
        bit = 0
        while bit < 8:
            if crc & 1:
                crc = (crc >> 1) ^ 0xedb88320
            else:
                crc >>= 1
            bit += 1
    return crc ^ 0xffffffff


def _crc_payload(data):
    """Return stable bytes independent of JSON key/spacing conventions."""
    parts = (
        str(data["format_version"]), str(data["sensor_id"]),
        str(data["image_width"]), str(data["image_height"]),
        str(data["crop_x"]), str(data["crop_y"]),
        str(data["crop_width"]), str(data["crop_height"]),
        str(data["grid_width"]), str(data["grid_height"]),
        str(data["reference_y"]),
        str(data["floor_y"]), str(data["reference_width_mm"]),
        str(data["accepted_frames"]), str(data["min_row_samples"]),
        ",".join(str(int(value)) for value in data["width_valid"]),
        ",".join(str(int(value)) for value in data["width_cells"]),
        ",".join(str(int(value)) for value in data["width_interpolated"]),
        ",".join(str(int(value)) for value in data["row_sample_count"]),
    )
    return ("ring-width-v2|" + "|".join(parts)).encode("ascii")


class RingWidthCalibration:
    """Collect, validate, persist, and expose one calibrated width row-map."""

    def __init__(self):
        self.width_valid = bytearray(HEIGHT)
        self.width_cells = array("i", [0] * HEIGHT)
        self.width_interpolated = bytearray(HEIGHT)
        # Compatibility aliases used by the runtime's fixed-grid contract.
        self.width_valid60 = self.width_valid
        self.width_cells60 = self.width_cells
        self.calibration_ready = False
        self.calibration_reason = "NO_CALIBRATION"
        self.ready = False
        self.reason = self.calibration_reason
        self.accepted_frames = 0
        self.valid_rows = 0
        # Row-major avoids wasting slots when one edge row is briefly absent:
        # each of the 52 useful rows has an independent 64-sample capacity.
        self._samples = array("H", [0] * (MAX_ROW_SAMPLES * HEIGHT))
        self._row_sample_count = bytearray(HEIGHT)
        self._candidate_valid = bytearray(HEIGHT)
        self.attempted_frames = 0
        self._current_valid = bytearray(HEIGHT)
        self._current_width = array("i", [0] * HEIGHT)
        self._current_center2 = array("i", [0] * HEIGHT)
        self._previous_valid = bytearray(HEIGHT)
        self._previous_width = array("i", [0] * HEIGHT)
        self._previous_center2 = array("i", [0] * HEIGHT)
        self._width_delta = array("i", [0] * HEIGHT)
        self._center_delta = array("i", [0] * HEIGHT)
        self._stable_streak = 0
        self._frame_valid_rows = 0
        self._frame_span = 0
        self._frame_center_rms_x2 = 0

    def _set_status(self, ready, reason):
        self.ready = bool(ready)
        self.calibration_ready = self.ready
        self.reason = reason
        self.calibration_reason = reason

    @property
    def collection_complete(self):
        if self.ready:
            return True
        if self.accepted_frames < REQUIRED_FRAMES:
            return False
        self._build_supported_map()
        return self._required_support_reason(self._candidate_valid) is None

    def _clear_current(self):
        for y in range(HEIGHT):
            self._current_valid[y] = 0
            self._current_width[y] = 0
            self._current_center2[y] = 0

    def _validate_frame(self, rows, count, stride):
        self._clear_current()
        if (not _is_int(count) or count < 1 or count > HEIGHT or
                not _is_int(stride) or stride < 4):
            return "INVALID_ROW_SHAPE"
        try:
            if len(rows) < count * stride:
                return "INVALID_ROW_SHAPE"
        except (TypeError, AttributeError):
            return "INVALID_ROW_SHAPE"

        present = 0
        mid_far = 0
        near = 0
        min_y = HEIGHT
        max_y = -1
        sum_y = 0
        sum_c2 = 0
        sum_yy = 0
        sum_yc2 = 0
        index = 0
        offset = 0
        while index < count:
            y = rows[offset]
            left = rows[offset + 1]
            right = rows[offset + 2]
            flags = rows[offset + 3]
            if (not _is_int(y) or not _is_int(left) or
                    not _is_int(right) or not _is_int(flags) or
                    (flags & 7) != 7 or y < FLOOR_Y or y >= HEIGHT or
                    self._current_valid[y] or left < 0 or
                    right >= WIDTH or right <= left):
                return "INVALID_ROW_GEOMETRY"
            width = right - left + 1
            if width < 8 or width > 70:
                return "WIDTH_OUT_OF_RANGE"
            self._current_valid[y] = 1
            self._current_width[y] = width
            center2 = left + right
            self._current_center2[y] = center2
            present += 1
            if y <= 47:
                mid_far += 1
            else:
                near += 1
            if y < min_y:
                min_y = y
            if y > max_y:
                max_y = y
            sum_y += y
            sum_c2 += center2
            sum_yy += y * y
            sum_yc2 += y * center2
            index += 1
            offset += stride

        if present < 20 or mid_far < 12 or near < 5 or max_y - min_y < 20:
            return "INSUFFICIENT_BILATERAL_ROWS"

        # A straight road may have perspective slope and camera yaw. Fit that
        # first-order trend, then reject curved centerlines by residual.
        denom = present * sum_yy - sum_y * sum_y
        if denom <= 0:
            return "BAD_STRAIGHT_FIT"
        slope = float(present * sum_yc2 - sum_y * sum_c2) / float(denom)
        intercept = float(sum_c2) / float(present) - \
            slope * float(sum_y) / float(present)
        residual_sq = 0.0
        max_residual = 0.0
        y = FLOOR_Y
        while y < HEIGHT:
            if self._current_valid[y]:
                residual = abs(float(self._current_center2[y]) -
                               (intercept + slope * y))
                residual_sq += residual * residual
                if residual > max_residual:
                    max_residual = residual
            y += 1
        rms = (residual_sq / float(present)) ** 0.5
        self._frame_valid_rows = present
        self._frame_span = max_y - min_y
        self._frame_center_rms_x2 = int(rms + 0.5)
        if max_residual > 6.0 or rms > 3.0:
            return "BAD_STRAIGHT_FIT"
        return None

    def _same_as_previous(self):
        common = 0
        width_delta = self._width_delta
        center_delta = self._center_delta
        for y in range(FLOOR_Y, HEIGHT):
            if self._current_valid[y] and self._previous_valid[y]:
                width_delta[common] = abs(
                    self._current_width[y] - self._previous_width[y])
                center_delta[common] = abs(
                    self._current_center2[y] - self._previous_center2[y])
                common += 1
        if common < 12:
            return False
        min_y = HEIGHT
        max_y = -1
        y = FLOOR_Y
        while y < HEIGHT:
            if self._current_valid[y] and self._previous_valid[y]:
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
            y += 1
        if max_y - min_y < 20:
            return False
        # Sort only bounded 52-element prefixes. A 1-cell road-width jitter
        # and <=1-pixel center jitter tolerate detector quantization.
        wi = 1
        while wi < common:
            value = width_delta[wi]
            at = wi
            while at > 0 and width_delta[at - 1] > value:
                width_delta[at] = width_delta[at - 1]
                at -= 1
            width_delta[at] = value
            wi += 1
        ci = 1
        while ci < common:
            value = center_delta[ci]
            at = ci
            while at > 0 and center_delta[at - 1] > value:
                center_delta[at] = center_delta[at - 1]
                at -= 1
            center_delta[at] = value
            ci += 1
        median = common // 2
        return (width_delta[median] <= 1 and width_delta[common - 1] <= 2 and
                center_delta[median] <= 2 and center_delta[common - 1] <= 4)

    def _build_supported_map(self):
        valid = self._candidate_valid
        for y in range(HEIGHT):
            valid[y] = 1 if self._row_sample_count[y] >= MIN_ROW_SAMPLES else 0
        y = FLOOR_Y
        while y < HEIGHT:
            if valid[y]:
                y += 1
                continue
            start = y
            while y < HEIGHT and not valid[y]:
                y += 1
            gap = y - start
            if (start > FLOOR_Y and y < HEIGHT and gap <= 3 and
                    valid[start - 1] and valid[y]):
                index = start
                while index < y:
                    valid[index] = 1
                    index += 1
        return valid

    @staticmethod
    def _required_support_reason(valid):
        support = 0
        min_y = HEIGHT
        max_y = -1
        for y in range(FLOOR_Y, HEIGHT):
            if valid[y]:
                support += 1
                if y < min_y:
                    min_y = y
                if y > max_y:
                    max_y = y
        if support < 12 or max_y - min_y < 20:
            return "INSUFFICIENT_CALIBRATION_CHAIN"
        nearest_delta = HEIGHT
        for y in range(FLOOR_Y, HEIGHT):
            if valid[y]:
                delta = abs(y - 40)
                if delta < nearest_delta:
                    nearest_delta = delta
        if nearest_delta > 3:
            return "REFERENCE_WIDTH_UNAVAILABLE"
        near_rows = 0
        near_run = 0
        max_near_run = 0
        for y in range(HEIGHT - 12, HEIGHT):
            if valid[y]:
                near_rows += 1
                near_run += 1
                if near_run > max_near_run:
                    max_near_run = near_run
            else:
                near_run = 0
        if near_rows < 5 or max_near_run < 4:
            return "NEAR_WIDTH_SUPPORT_INSUFFICIENT"
        return None

    def _copy_previous(self):
        for y in range(HEIGHT):
            self._previous_valid[y] = self._current_valid[y]
            self._previous_width[y] = self._current_width[y]
            self._previous_center2[y] = self._current_center2[y]

    def accept_rows(self, rows, count, stride):
        """Consider one fresh accepted bilateral row set for calibration.

        Row records use ``[y, left_x, right_x, ...]`` in 80x60 grid-cell
        coordinates.  The inclusive measured width is ``right-left+1``.
        The caller must already have established that the car is parked and
        the road topology is clear.  This method adds its own geometry and
        temporal-stability checks.
        """
        if self.ready:
            self._set_status(True, "CALIBRATION_READY")
            return False
        if (self.accepted_frames >= REQUIRED_FRAMES and
                self.collection_complete):
            self._set_status(False, "FINALIZE_REQUIRED")
            return False
        if self.attempted_frames >= MAX_ATTEMPTS:
            self._set_status(False, "COLLECTION_TIMEOUT")
            return False
        self.attempted_frames += 1
        failure = self._validate_frame(rows, count, stride)
        if failure is not None:
            self._stable_streak = 0
            self._copy_previous()
            self._set_status(False, failure)
            return False
        stable = self._same_as_previous()
        self._stable_streak = self._stable_streak + 1 if stable else 1
        self._copy_previous()
        if self._stable_streak < 3:
            self._set_status(False, "WAITING_FOR_STABLE_STRAIGHT")
            return False
        for y in range(FLOOR_Y, HEIGHT):
            sample_count = int(self._row_sample_count[y])
            if self._current_valid[y] and sample_count < MAX_ROW_SAMPLES:
                self._samples[y * MAX_ROW_SAMPLES + sample_count] = \
                    self._current_width[y]
                self._row_sample_count[y] = sample_count + 1
        self.accepted_frames += 1
        self._set_status(False, "READY_TO_FINALIZE" if
                         self.collection_complete else "COLLECTING")
        return True

    @staticmethod
    def _sort_prefix(values, count):
        index = 1
        while index < count:
            value = values[index]
            at = index
            while at > 0 and values[at - 1] > value:
                values[at] = values[at - 1]
                at -= 1
            values[at] = value
            index += 1

    def finalize(self):
        """Build and return the fixed-grid calibration payload."""
        for y in range(HEIGHT):
            self.width_valid[y] = 0
            self.width_cells[y] = 0
            self.width_interpolated[y] = 0
        if self.accepted_frames < REQUIRED_FRAMES:
            self.valid_rows = 0
            self._set_status(False, "INSUFFICIENT_STABLE_FRAMES")
            return self._payload()

        scratch = array("H", [0] * MAX_ROW_SAMPLES)
        valid_rows = 0
        for y in range(FLOOR_Y, HEIGHT):
            count = int(self._row_sample_count[y])
            if count < MIN_ROW_SAMPLES:
                continue
            sample = 0
            base = y * MAX_ROW_SAMPLES
            while sample < count:
                scratch[sample] = self._samples[base + sample]
                sample += 1
            if sample < MIN_ROW_SAMPLES:
                continue
            self._sort_prefix(scratch, sample)
            middle = sample // 2
            if sample & 1:
                median = int(scratch[middle])
            else:
                median = (int(scratch[middle - 1]) +
                          int(scratch[middle]) + 1) // 2
            if median < 8 or median > 70:
                continue
            self.width_valid[y] = 1
            self.width_cells[y] = median
            self.width_interpolated[y] = 0
            valid_rows += 1

        # Fill only short internal row gaps. No endpoint extrapolation or
        # guessed width is permitted.
        y = FLOOR_Y
        while y < HEIGHT:
            if self.width_valid[y]:
                y += 1
                continue
            start = y
            while y < HEIGHT and not self.width_valid[y]:
                y += 1
            gap = y - start
            if (start > FLOOR_Y and y < HEIGHT and gap <= 3 and
                    self.width_valid[start - 1] and self.width_valid[y]):
                left = int(self.width_cells[start - 1])
                right = int(self.width_cells[y])
                offset = 0
                while offset < gap:
                    numerator = left * (gap - offset) + right * (offset + 1)
                    self.width_cells[start + offset] = (
                        numerator + gap // 2) // (gap + 1)
                    self.width_valid[start + offset] = 1
                    self.width_interpolated[start + offset] = 1
                    offset += 1
                    valid_rows += 1

        self.valid_rows = valid_rows
        support_reason = self._required_support_reason(self.width_valid)
        if support_reason is not None:
            self._set_status(False, support_reason)
            return self._payload()
        self._set_status(True, "CALIBRATION_READY")
        return self._payload()

    def _payload(self):
        return {
            "format_version": FORMAT_VERSION,
            "sensor_id": SENSOR_ID,
            "image_width": CROP_WIDTH,
            "image_height": CROP_HEIGHT,
            "crop_x": CROP_X,
            "crop_y": CROP_Y,
            "crop_width": CROP_WIDTH,
            "crop_height": CROP_HEIGHT,
            "grid_width": WIDTH,
            "grid_height": HEIGHT,
            "reference_y": REFERENCE_Y,
            "floor_y": FLOOR_Y,
            "reference_width_mm": REFERENCE_WIDTH_MM,
            "accepted_frames": self.accepted_frames,
            "attempted_frames": self.attempted_frames,
            "collection_complete": self.collection_complete,
            "min_row_samples": MIN_ROW_SAMPLES,
            "width_valid": list(self.width_valid),
            "width_cells": list(self.width_cells),
            "width_interpolated": list(self.width_interpolated),
            "row_sample_count": list(self._row_sample_count),
            "ready": self.ready,
            "reason": self.reason,
        }

    def save(self, path):
        """Finalize and write a CRC-protected private calibration file."""
        data = self.finalize()
        if not data["ready"]:
            raise ValueError("ring width calibration is not ready: %s" %
                             data["reason"])
        crc = _u32_crc(_crc_payload(data))
        data["crc32"] = "%08X" % crc
        target = str(path)
        temporary = target + ".tmp"
        try:
            with open(temporary, "w") as destination:
                destination.write(_json.dumps(data))
                flush = getattr(destination, "flush", None)
                if flush is not None:
                    flush()
            replace = getattr(os, "replace", None)
            if replace is not None:
                replace(temporary, target)
            else:
                os.rename(temporary, target)
        except Exception:
            try:
                os.remove(temporary)
            except OSError:
                pass
            raise
        return data

    @classmethod
    def load(cls, path):
        """Load calibration; missing or invalid files return a not-ready map."""
        calibration = cls()
        try:
            with open(path, "r") as source:
                data = _json.loads(source.read())
        except OSError:
            calibration._set_status(False, "FILE_NOT_FOUND")
            return calibration
        except Exception:
            calibration._set_status(False, "INVALID_FILE")
            return calibration
        try:
            expected = {
                "format_version": FORMAT_VERSION,
                "sensor_id": SENSOR_ID,
                "image_width": CROP_WIDTH,
                "image_height": CROP_HEIGHT,
                "crop_x": CROP_X,
                "crop_y": CROP_Y,
                "crop_width": CROP_WIDTH,
                "crop_height": CROP_HEIGHT,
                "grid_width": WIDTH,
                "grid_height": HEIGHT,
                "reference_y": REFERENCE_Y,
                "floor_y": FLOOR_Y,
                "reference_width_mm": REFERENCE_WIDTH_MM,
                "min_row_samples": MIN_ROW_SAMPLES,
            }
            for key, value in expected.items():
                if data.get(key) != value:
                    calibration._set_status(False, "GEOMETRY_MISMATCH")
                    return calibration
            valid = data.get("width_valid")
            widths = data.get("width_cells")
            interpolated = data.get("width_interpolated")
            sample_counts = data.get("row_sample_count")
            frames = data.get("accepted_frames")
            if (not isinstance(valid, list) or len(valid) != HEIGHT or
                    not isinstance(widths, list) or len(widths) != HEIGHT or
                    not isinstance(interpolated, list) or
                    len(interpolated) != HEIGHT or
                    not isinstance(sample_counts, list) or
                    len(sample_counts) != HEIGHT or
                    not _is_int(frames) or frames < REQUIRED_FRAMES):
                calibration._set_status(False, "INVALID_CALIBRATION_DATA")
                return calibration
            for y in range(HEIGHT):
                flag = valid[y]
                width = widths[y]
                was_interpolated = interpolated[y]
                samples = sample_counts[y]
                if (not _is_int(flag) or flag not in (0, 1) or
                        not _is_int(width) or
                        not _is_int(was_interpolated) or
                        was_interpolated not in (0, 1) or
                        not _is_int(samples) or samples < 0 or
                        samples > MAX_ROW_SAMPLES):
                    calibration._set_status(False, "INVALID_CALIBRATION_DATA")
                    return calibration
                if y < FLOOR_Y:
                    if flag or width or was_interpolated or samples:
                        calibration._set_status(False, "INVALID_CALIBRATION_DATA")
                        return calibration
                elif (flag == 0 and (width or was_interpolated or
                                     samples >= MIN_ROW_SAMPLES)):
                    calibration._set_status(False, "INVALID_CALIBRATION_DATA")
                    return calibration
                elif flag == 1 and (width < 8 or width > 70 or
                                    (was_interpolated and
                                     samples >= MIN_ROW_SAMPLES) or
                                    (not was_interpolated and
                                     samples < MIN_ROW_SAMPLES)):
                    calibration._set_status(False, "INVALID_CALIBRATION_DATA")
                    return calibration
                calibration.width_valid[y] = flag
                calibration.width_cells[y] = width
                calibration.width_interpolated[y] = was_interpolated
                calibration._row_sample_count[y] = samples
            # An interpolated value must be an internal run no longer than
            # three rows with measured endpoints; no edge extrapolation.
            y = FLOOR_Y
            while y < HEIGHT:
                if not calibration.width_interpolated[y]:
                    y += 1
                    continue
                start = y
                while y < HEIGHT and calibration.width_interpolated[y]:
                    y += 1
                gap = y - start
                if (gap > 3 or start <= FLOOR_Y or y >= HEIGHT or
                        not calibration.width_valid[start - 1] or
                        calibration.width_interpolated[start - 1] or
                        not calibration.width_valid[y] or
                        calibration.width_interpolated[y]):
                    calibration._set_status(False, "INVALID_CALIBRATION_DATA")
                    return calibration
            support_reason = cls._required_support_reason(
                calibration.width_valid)
            if support_reason is not None:
                calibration._set_status(False, support_reason)
                return calibration
            if int(data.get("crc32", -1), 16) != _u32_crc(
                    _crc_payload(data)):
                calibration._set_status(False, "CRC_MISMATCH")
                return calibration
            calibration.accepted_frames = frames
            calibration.attempted_frames = int(data.get(
                "attempted_frames", frames))
            calibration.valid_rows = sum(calibration.width_valid)
            calibration._set_status(True, "CALIBRATION_READY")
            return calibration
        except Exception:
            calibration._set_status(False, "INVALID_CALIBRATION_DATA")
            return calibration

