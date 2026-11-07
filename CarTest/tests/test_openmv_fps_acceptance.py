import csv
import io
import unittest

from tools import check_openmv_fps_acceptance as acceptance


def board_line(index, *, captures=601, writes=601, valid=580, errors=0,
               attempts=None, run=7, start_ms=None, end_ms=None):
    if start_ms is None:
        start_ms = index * acceptance.WINDOW_MS
    if end_ms is None:
        end_ms = start_ms + acceptance.WINDOW_MS
    if attempts is None:
        attempts = writes
    return (
        "V7_PERF_WINDOW run={run} index={index} start_ms={start} end_ms={end} "
        "distinct_captures={captures} write_attempts={attempts} "
        "full_uart_writes={writes} valid_frames={valid} "
        "valid_full_writes={valid} late_invalid_frames=0 "
        "uart_write_errors={errors}\n"
    ).format(
        run=run, index=index, start=start_ms, end=end_ms,
        captures=captures, attempts=attempts, writes=writes,
        valid=valid, errors=errors)


def board_log(indices=range(6), **kwargs):
    return "".join(board_line(index, **kwargs) for index in indices)


def preview_csv(count=601, indices=range(6)):
    output = io.StringIO(newline="")
    writer = csv.writer(output, lineterminator="\n")
    writer.writerow(acceptance.PREVIEW_REQUIRED_COLUMNS)
    for index in indices:
        start_ms = index * acceptance.WINDOW_MS
        writer.writerow((start_ms, start_ms + acceptance.WINDOW_MS, count))
    return output.getvalue()


class OpenMvFpsAcceptanceTests(unittest.TestCase):
    def assess(self, text=None, **kwargs):
        return acceptance.evaluate(
            board_log() if text is None else text,
            scenario=kwargs.pop("scenario", "no-preview"),
            board_evidence=kwargs.pop("board_evidence", True),
            **kwargs)

    def test_exactly_60_fps_does_not_pass(self):
        result = self.assess(board_log(captures=600, writes=600))
        self.assertEqual(result["status"], "FAIL")
        self.assertTrue(all(window["metric_status"] == "FAIL"
                            for window in result["board"]["windows"]))
        self.assertTrue(all(window["rates"]["distinct_capture_fps"] == 60.0
                            for window in result["board"]["windows"]))

    def test_uart_rate_is_checked_independently_from_capture_rate(self):
        result = self.assess(board_log(captures=601, writes=600))
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("full_uart_write_rate_not_strictly_above_60_fps",
                      result["board"]["windows"][0]["reasons"])

    def test_full_writes_cannot_exceed_recorded_write_attempts(self):
        result = self.assess(board_log(attempts=600, writes=601))
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("full_uart_writes_exceed_write_attempts",
                      result["board"]["windows"][0]["reasons"])

    def test_strictly_above_60_fps_passes_and_validity_stays_separate(self):
        result = self.assess(board_log(valid=0))
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["board"]["windows"][0]["metric_status"], "PASS")
        self.assertEqual(result["board"]["windows"][0]["rates"]["valid_frame_fps"], 0.0)
        self.assertTrue(result["board"]["validity_is_reported_separately"])

    def test_ide_preview_without_independent_new_frame_counts_is_incomplete(self):
        result = self.assess(scenario="ide-preview", preview_csv_text=None)
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertEqual(result["preview"]["status"], "INCOMPLETE")
        self.assertIn("independent_preview_evidence_missing",
                      [issue["kind"] for issue in result["preview"]["issues"]])

    def test_aligned_ide_preview_above_60_fps_passes(self):
        result = self.assess(
            scenario="ide-preview",
            preview_csv_text=preview_csv(601))
        self.assertEqual(result["status"], "PASS")
        self.assertEqual(result["preview"]["status"], "PASS")
        self.assertFalse(result["preview"]["host_video_fps_used"])

    def test_ide_preview_at_exactly_60_fps_fails(self):
        result = self.assess(
            scenario="ide-preview",
            preview_csv_text=preview_csv(600))
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["preview"]["windows"][0]["status"], "FAIL")

    def test_duplicate_and_reversed_board_windows_fail_closed(self):
        duplicate = self.assess(board_log((0, 1, 2, 3, 4, 4)))
        self.assertEqual(duplicate["status"], "FAIL")
        self.assertIn("duplicate_board_window",
                      [issue["kind"] for issue in duplicate["board"]["issues"]])

        reversed_order = self.assess(board_log((0, 1, 3, 2, 4, 5)))
        self.assertEqual(reversed_order["status"], "FAIL")
        self.assertIn("reversed_board_window_order",
                      [issue["kind"] for issue in reversed_order["board"]["issues"]])

    def test_missing_window_does_not_accumulate_a_60_second_streak(self):
        result = self.assess(board_log((0, 1, 3, 4, 5)))
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertEqual(result["board"]["longest_consecutive_10s_streak"], 3)
        self.assertIn("board_window_gap",
                      [issue["kind"] for issue in result["board"]["issues"]])

    def test_write_errors_fail_even_when_rates_are_above_60(self):
        result = self.assess(board_log(errors=1))
        self.assertEqual(result["status"], "FAIL")
        self.assertIn("uart_write_errors_not_zero",
                      result["board"]["windows"][0]["reasons"])

    def test_old_field_names_are_not_accepted_as_current_board_evidence(self):
        old_log = "".join(
            "V7_PERF_WINDOW run=7 index={index} start_ms={start} end_ms={end} "
            "frames=601 full_writes=601 valid_frames=600 write_errors=0\n".format(
                index=index, start=index * acceptance.WINDOW_MS,
                end=(index + 1) * acceptance.WINDOW_MS)
            for index in range(6))
        result = self.assess(old_log)
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertIn("missing_required_fields:distinct_captures,write_attempts,full_uart_writes,uart_write_errors",
                      result["board"]["windows"][0]["reasons"])

    def test_bad_preview_csv_fails_closed(self):
        malformed = "start_ms,end_ms,frames\n0,10000,700\n"
        result = self.assess(scenario="ide-preview", preview_csv_text=malformed)
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["preview"]["status"], "FAIL")
        self.assertIn("missing_preview_csv_columns",
                      [issue["kind"] for issue in result["preview"]["issues"]])

        extra_field = preview_csv(601).replace("0,10000,601", "0,10000,601,extra", 1)
        extra_result = self.assess(
            scenario="ide-preview", preview_csv_text=extra_field)
        self.assertEqual(extra_result["status"], "FAIL")
        self.assertIn("malformed_preview_csv_row",
                      [issue["kind"] for issue in extra_result["preview"]["issues"]])

    def test_preview_time_windows_must_exactly_cover_board_windows(self):
        original = preview_csv(601).splitlines()
        shifted = "\n".join([original[0]] + [
            "{},{},{}".format(int(parts[0]) + 1, int(parts[1]) + 1, parts[2])
            for parts in (line.split(",") for line in original[1:])
        ]) + "\n"
        result = self.assess(scenario="ide-preview", preview_csv_text=shifted)
        self.assertEqual(result["status"], "INCOMPLETE")

    def test_duplicate_and_reversed_preview_windows_fail_closed(self):
        duplicate_rows = list(range(6)) + [5]
        duplicate_result = self.assess(
            scenario="ide-preview",
            preview_csv_text=preview_csv(601, duplicate_rows))
        self.assertEqual(duplicate_result["status"], "FAIL")
        self.assertIn("duplicate_preview_window",
                      [issue["kind"] for issue in duplicate_result["preview"]["issues"]])

        reversed_rows = [0, 1, 3, 2, 4, 5]
        reversed_result = self.assess(
            scenario="ide-preview",
            preview_csv_text=preview_csv(601, reversed_rows))
        self.assertEqual(reversed_result["status"], "FAIL")
        self.assertIn("reversed_preview_window_order",
                      [issue["kind"] for issue in reversed_result["preview"]["issues"]])

    def test_board_source_flag_is_required_for_pass(self):
        result = self.assess(board_evidence=False)
        self.assertEqual(result["status"], "INCOMPLETE")
        self.assertEqual(result["inputs"]["board_evidence_attested_by_operator"], False)
        self.assertIn("board_source_not_attested_by_operator", result["reasons"])

    def test_minimum_window_requirement_cannot_be_lowered(self):
        with self.assertRaises(ValueError):
            acceptance.evaluate(board_log(), scenario="no-preview", min_windows=5)

    def test_log_traceback_fails_runtime_integrity(self):
        result = self.assess(board_log() + "Traceback (most recent call last):\n"
                             "AttributeError: frame output unavailable\n")
        self.assertEqual(result["status"], "FAIL")
        self.assertEqual(result["board"]["runtime_errors"][0]["kind"],
                         "board_runtime_traceback")


if __name__ == "__main__":
    unittest.main()
