import csv
import hashlib
import json
import sys
import tempfile
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT_ROOT / "tools"))
import analyze_differential_runs as analyzer  # noqa: E402
import runtime_log_export as exporter  # noqa: E402


SYNTHETIC_LABEL = "SYNTHETIC TEST DATA ONLY; NOT VEHICLE OBSERVATIONS"
SYNTHETIC_TRIAL_SET_ID = "analyzer-host-tests-v1"
U32_MASK = 0xFFFFFFFF
TIMES_MS = [0, 50, 100, 150, 250, 300, 350, 450, 500]
HOLD_TIMES = {150, 250, 350}


def decode_synthetic_host_fixture(_elf_path, raw_path):
    """Test-only mock decoder for explicitly labeled JSON host fixtures."""
    document = json.loads(Path(raw_path).read_text(encoding="utf-8"))
    if document.get("synthetic_host_fixture") != SYNTHETIC_LABEL:
        raise ValueError("not an explicitly labeled synthetic host fixture")
    return None, document["header"], document["rows"]


def sync_synthetic_fixture(path, rows=None, *, header_updates=None):
    """Rewrite CSV and labeled host raw fixture together for integrity-aware tests."""
    metadata_path = path / "run-metadata.json"
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
    raw_path = path / "runtime-log.bin"
    document = json.loads(raw_path.read_text(encoding="utf-8"))
    if rows is None:
        rows = document["rows"]
    header = document["header"]
    if header_updates:
        header.update(header_updates)
    header["count"] = len(rows)
    for row in rows:
        row["t_elapsed_ms"] = (row["t_ms"] - header["start_ms"]) & U32_MASK
    document["header"] = header
    document["rows"] = rows
    raw_bytes = json.dumps(document, sort_keys=True).encode("utf-8")
    raw_path.write_bytes(raw_bytes)
    with (path / "run.csv").open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=exporter.CSV_FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    metadata["initial_header"] = header
    metadata["record_count"] = len(rows)
    metadata["raw_sha256"] = hashlib.sha256(raw_bytes).hexdigest()
    metadata["dropped_count"] = header["dropped_count"]
    metadata["buffer_full"] = header["buffer_full"]
    metadata["records_truncated"] = header["records_truncated"]
    metadata_path.write_text(json.dumps(metadata), encoding="utf-8")


def reschedule_synthetic_fixture(path, elapsed_values):
    document = json.loads((path / "runtime-log.bin").read_text(encoding="utf-8"))
    rows = document["rows"]
    if len(rows) != len(elapsed_values):
        raise AssertionError("schedule length must match fixture record count")
    start_ms = document["header"]["start_ms"]
    for row, elapsed in zip(rows, elapsed_values):
        row["t_ms"] = (start_ms + elapsed) & U32_MASK
    stop_ms = (start_ms + elapsed_values[-1]) & U32_MASK
    sync_synthetic_fixture(path, rows, header_updates={"stop_ms": stop_ms})


class DifferentialAnalysisTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        self.artifacts_path = self.root / "ARTIFACTS.json"
        self._write_artifacts()

    def tearDown(self):
        self.temp.cleanup()

    @staticmethod
    def _artifact_data():
        profiles = {}
        for index, (variant, diff) in enumerate(analyzer.PROFILE_DIFF_PERMILLE.items(), start=1):
            profiles[variant] = {
                "variant": variant,
                "diff_permille": diff,
                "elf_sha256": f"{index:064x}",
                "build_id": f"{index:032x}",
                "servo_center_us": 1530,
            }
        return {"schema_version": 1, "profiles": profiles}

    def _write_artifacts(self):
        self.artifacts = self._artifact_data()
        self.artifacts_path.write_text(json.dumps(self.artifacts), encoding="utf-8")

    @staticmethod
    def _conditions(**overrides):
        result = {
            "layout_id": "layout-A",
            "camera_build_id": "camera-A",
            "battery_voltage_v": 7.60,
            "battery_state": "charged-pack-1",
            "lighting_id": "light-A",
            "lighting_notes": "synthetic fixture",
        }
        result.update(overrides)
        return result

    def _make_rows(self, variant, session_id, start_ms=1000, mutate=None):
        error_bias = {"none": 700, "medium": 350, "strong": 100}[variant]
        servo_limited_gap = {"none": 8, "medium": 5, "strong": 2}[variant]
        servo_applied_gap = {"none": 12, "medium": 7, "strong": 3}[variant]
        rows = []
        for index, elapsed in enumerate(TIMES_MS):
            hold = elapsed in HOLD_TIMES
            final = index == len(TIMES_MS) - 1
            start = index == 0
            row = {field: 0 for field in exporter.CSV_FIELDS}
            row.update(
                {
                    "t_ms": (start_ms + elapsed) & U32_MASK,
                    "t_elapsed_ms": elapsed,
                    "session_id": session_id,
                    "encoder_left_count": 100 * index,
                    "encoder_right_count": 95 * index,
                    "fault_generation": 0,
                    "servo_requested_us": 0 if final else 1530 + 80,
                    "servo_limited_us": 0 if final else 1530 + 80 - servo_limited_gap,
                    "servo_applied_us": 0 if final else 1530 + 80 - servo_applied_gap,
                    "motor_requested_permille": 0 if final else 500,
                    "motor_left_applied_permille": 0 if final else 500,
                    "motor_right_applied_permille": 0 if final else 500,
                    "tim3_ccr1_counts": 0 if final else 2400,
                    "tim3_ccr2_counts": 0 if final else 2375,
                    "tim3_ccr3_counts": 0 if final else 1530,
                    "tim3_ccr4_counts": 0,
                    "vision_latest_error_norm_milli": (
                        error_bias if elapsed in (300, 450) else 0
                    ),
                    "vision_sequence": index,
                    "vision_confidence_percent": 0,
                    "vision_flags": 0,
                    "vision_latest_valid": int(not hold and not final),
                    "vision_usable_fresh": int(not hold and not final),
                    "vision_timeout": 0,
                    "vision_latest_frame_present": int(not hold and not final),
                    "vision_hold_active": int(hold),
                    "vision_hold_expired": 0,
                    "vision_hold_elapsed_ms": 0,
                    "vision_hold_valid_count": 0,
                    "auto_recovery_count": 0,
                    "recovery_frame_count": 0,
                    "recovery_active": 0,
                    "mode": 0 if final else (12 if hold else 10),
                    "fault": 0,
                    "event_flags": 0,
                    "event_periodic": 1,
                    "event_start": int(start),
                    "event_stop": int(final),
                    "event_fault": 0,
                    "event_mode": 0,
                    "event_recovery": 0,
                    "event_auto_recovery": 0,
                    "event_hold_enter": int(elapsed in (150, 350)),
                    "event_hold_exit": int(elapsed in (300, 450)),
                    "event_hold_deadline": 0,
                    "event_button_start": int(start),
                    "event_button_stop": int(final),
                    "vision_hold_exit_cause": 0,
                }
            )
            if mutate:
                mutate(row, index, elapsed)
            rows.append(row)
        return rows

    def _make_run(
        self,
        variant,
        number,
        *,
        run_id=None,
        session_id=None,
        start_ms=1000,
        conditions=None,
        completed=True,
        out_of_track=False,
        mutate_row=None,
        metadata_mutate=None,
        path_name=None,
    ):
        session_id = session_id if session_id is not None else number
        path = self.root / (path_name or f"runs/{variant}-{number}")
        path.mkdir(parents=True, exist_ok=True)
        rows = self._make_rows(variant, session_id, start_ms, mutate_row)
        csv_path = path / "run.csv"
        with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=exporter.CSV_FIELDS)
            writer.writeheader()
            writer.writerows(rows)

        profile = self.artifacts["profiles"][variant]
        start_u32 = start_ms & U32_MASK
        stop_u32 = (start_ms + TIMES_MS[-1]) & U32_MASK
        header = {
            "state": analyzer.STATE_FROZEN,
            "state_name": "frozen",
            "session_id": session_id,
            "period_ms": 50,
            "start_ms": start_u32,
            "stop_ms": stop_u32,
            "stop_reason": 1,
            "count": len(rows),
            "dropped_count": 0,
            "buffer_full": False,
            "records_truncated": False,
            "build_id": profile["build_id"],
        }
        metadata = {
            "status": "success",
            "stage": "complete",
            "elf_sha256": profile["elf_sha256"],
            "build_id": profile["build_id"],
            "initial_header": header,
            "record_count": len(rows),
            "dropped_count": 0,
            "buffer_full": False,
            "records_truncated": False,
            "raw_sha256": "",
        }
        if metadata_mutate:
            metadata_mutate(metadata, header)
        raw_bytes = json.dumps(
            {
                "synthetic_host_fixture": SYNTHETIC_LABEL,
                "header": header,
                "rows": rows,
            },
            sort_keys=True,
        ).encode("utf-8")
        (path / "runtime-log.bin").write_bytes(raw_bytes)
        if metadata.get("raw_sha256") == "":
            metadata["raw_sha256"] = hashlib.sha256(raw_bytes).hexdigest()
        (path / "run-metadata.json").write_text(json.dumps(metadata), encoding="utf-8")
        entry = {
            "trial_set_id": SYNTHETIC_TRIAL_SET_ID,
            "run_id": run_id or f"{variant}-{number}",
            "path": str(path),
            "variant": variant,
            "completed": completed,
            "out_of_track": out_of_track,
            **self._conditions(**(conditions or {})),
        }
        return entry, path

    def _write_manifest(
        self,
        runs,
        *,
        dataset_type="synthetic-test-only",
        trial_set_id=SYNTHETIC_TRIAL_SET_ID,
        path=None,
    ):
        manifest_path = path or (self.root / "manifest.json")
        manifest_path.write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "dataset_type": dataset_type,
                    "trial_set_id": trial_set_id,
                    "description": SYNTHETIC_LABEL,
                    "runs": runs,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        return manifest_path

    def _balanced_runs(self, *, completed=True, out_of_track=False):
        result = []
        for variant in analyzer.PROFILE_DIFF_PERMILLE:
            for number in range(1, 4):
                entry, _ = self._make_run(
                    variant,
                    number + 100 * list(analyzer.PROFILE_DIFF_PERMILLE).index(variant),
                    completed=completed,
                    out_of_track=out_of_track,
                )
                result.append(entry)
        return result

    def _analyze(self, runs, *, diagnostics_root=None, startup_exclude_ms=None):
        manifest = self._write_manifest(runs)
        return analyzer.analyze_manifest(
            manifest,
            self.artifacts_path,
            diagnostics_root,
            startup_exclude_ms,
            None,
            synthetic_decoder=decode_synthetic_host_fixture,
        )

    def test_three_profiles_get_synthetic_order_but_never_real_recommendation(self):
        result = self._analyze(self._balanced_runs())
        self.assertEqual(result["analysis_status"], "synthetic-test-only")
        self.assertIsNone(result["recommendation"])
        self.assertTrue(result["synthetic_order_is_not_a_vehicle_recommendation"])
        order = {item["variant"]: item["synthetic_order"] for item in result["profiles"]}
        self.assertEqual(order, {"none": 3, "medium": 2, "strong": 1})
        self.assertTrue(all(item["eligible"] for item in result["profiles"]))

    def test_fourth_failed_attempt_cannot_replace_exact_three_attempt_rule(self):
        runs = self._balanced_runs()
        extra, _ = self._make_run("none", 999, completed=False)
        runs.append(extra)
        result = self._analyze(runs)
        none = next(item for item in result["profiles"] if item["variant"] == "none")
        self.assertEqual(none["attempted_runs"], 4)
        self.assertEqual(none["profile_status"], "insufficient")
        self.assertFalse(result["trial_set_complete"])
        self.assertTrue(all(item["synthetic_order"] is None for item in result["profiles"]))

    def test_one_failed_profile_does_not_block_other_complete_profiles(self):
        runs = self._balanced_runs()
        runs[0]["completed"] = False
        result = self._analyze(runs)
        profiles = {item["variant"]: item for item in result["profiles"]}
        self.assertTrue(result["trial_set_complete"])
        self.assertEqual(profiles["none"]["profile_status"], "failed")
        self.assertEqual(profiles["none"]["profile_eligible_runs"], 2)
        self.assertEqual(profiles["medium"]["profile_status"], "passed")
        self.assertEqual(profiles["strong"]["profile_status"], "passed")
        self.assertIsNone(profiles["none"]["synthetic_order"])
        self.assertIsNotNone(profiles["medium"]["synthetic_order"])
        self.assertIsNotNone(profiles["strong"]["synthetic_order"])
        self.assertIsNone(result["recommendation"])

    def test_all_verified_profiles_failed_reports_no_eligible_profile(self):
        result = self._analyze(self._balanced_runs(completed=False))
        self.assertTrue(result["trial_set_complete"])
        self.assertEqual(result["trial_set_status"], "no_eligible_profile")
        self.assertTrue(all(item["profile_status"] == "failed" for item in result["profiles"]))
        self.assertTrue(all(item["synthetic_order"] is None for item in result["profiles"]))
        self.assertIsNone(result["recommendation"])

    def test_verified_startup_hardfault_without_road_ready_is_failed_not_insufficient(self):
        def stopped_at_start(row, _index, _elapsed):
            row["mode"] = 8
            row["fault"] = 1
            row["vision_latest_valid"] = 0
            row["vision_usable_fresh"] = 0
            row["motor_requested_permille"] = 0
            row["motor_left_applied_permille"] = 0
            row["motor_right_applied_permille"] = 0
            row["servo_applied_us"] = 0
            for channel in (1, 2, 3, 4):
                row[f"tim3_ccr{channel}_counts"] = 0

        entry, _ = self._make_run("none", 1, completed=False, mutate_row=stopped_at_start)
        result = self._analyze([entry])
        run = result["runs"][0]
        self.assertTrue(run["export_valid"])
        self.assertTrue(run["hard_fault"])
        self.assertTrue(run["group_data_valid"])
        self.assertEqual(run["outcome"], "failed")
        self.assertIsNone(run["metrics"])

    def test_report_and_machine_outputs_are_written_and_labeled_synthetic(self):
        result = self._analyze(self._balanced_runs())
        output = self.root / "output"
        analyzer._write_outputs(output, result)
        self.assertTrue((output / "summary.csv").is_file())
        self.assertTrue((output / "analysis.json").is_file())
        report = (output / "report.md").read_text(encoding="utf-8")
        template = json.loads((output / "manual-metadata-template.json").read_text(encoding="utf-8"))
        report_lines = report.splitlines()
        header_line = next(line for line in report_lines if line.startswith("| 档位 |"))
        separator_line = report_lines[report_lines.index(header_line) + 1]
        first_profile_line = report_lines[report_lines.index(header_line) + 2]
        self.assertEqual(len(header_line.split("|")), len(separator_line.split("|")))
        self.assertEqual(len(header_line.split("|")), len(first_profile_line.split("|")))
        self.assertIn("合成测试数据", report)
        self.assertIn("不代表实车观察", report)
        self.assertEqual(template["dataset_type"], "real")
        self.assertEqual(template["runs"], [])
        self.assertIsNone(template["run_entry_template"]["completed"])
        self.assertIsNone(template["run_entry_template"]["out_of_track"])

    def test_time_weighted_hold_and_error_use_interval_duration_not_event_rows(self):
        entry, _ = self._make_run("none", 1)
        result = self._analyze([entry])
        run = result["runs"][0]
        metrics = run["metrics"]
        self.assertEqual(metrics["window_duration_ms"], 500)
        self.assertEqual(metrics["hold_active_ms"], 250)
        self.assertEqual(metrics["hold_enter_event_count"], 2)
        self.assertEqual(metrics["reliable_route_ms"], 250)
        self.assertEqual(metrics["reliable_abs_error_norm_milli_mean"], 280)
        self.assertEqual(metrics["road_ready_elapsed_ms"], 0)
        self.assertTrue(run["eligible_for_profile"])
        self.assertEqual(metrics["vision_timeout_ms"], 0)

    def test_startup_waits_for_delayed_550_450_road_ready_and_weights_irregular_intervals(self):
        entry, path = self._make_run("none", 1)
        reschedule_synthetic_fixture(path, [0, 50, 125, 200, 275, 325, 475, 550, 650])
        document = json.loads((path / "runtime-log.bin").read_text(encoding="utf-8"))
        rows = document["rows"]
        for index, row in enumerate(rows[:-1]):
            row.update(
                {
                    "mode": 10,
                    "vision_latest_valid": 1,
                    "vision_usable_fresh": 1,
                    "vision_timeout": 0,
                    "vision_hold_active": 0,
                    "vision_hold_expired": 0,
                    "motor_requested_permille": 500,
                    "servo_requested_us": 1600,
                    "servo_limited_us": 1590,
                    "servo_applied_us": 1580,
                    "motor_left_applied_permille": 0 if index < 4 else 500,
                    "motor_right_applied_permille": 0 if index < 4 else 500,
                    "tim3_ccr1_counts": 0 if index < 4 else 2400,
                    "tim3_ccr2_counts": 0 if index < 4 else 2375,
                    "vision_latest_error_norm_milli": 0,
                    "event_hold_enter": 0,
                    "event_hold_exit": 0,
                    "event_hold_deadline": 0,
                }
            )
        rows[3]["vision_hold_active"] = 1
        rows[3]["mode"] = 12
        rows[4]["motor_left_applied_permille"] = 550
        rows[4]["motor_right_applied_permille"] = 450
        rows[4]["tim3_ccr1_counts"] = 2400
        rows[4]["tim3_ccr2_counts"] = 2375
        for index, error in ((4, 100), (5, 300), (6, 500), (7, 700)):
            rows[index]["vision_latest_error_norm_milli"] = error
        rows[-1]["mode"] = 0
        for field in analyzer.OUTPUT_ZERO_FIELDS:
            rows[-1][field] = 0
        header_stop = (document["header"]["start_ms"] + 650) & U32_MASK
        sync_synthetic_fixture(path, rows, header_updates={"stop_ms": header_stop})

        result = self._analyze([entry])
        run = result["runs"][0]
        metrics = run["metrics"]
        self.assertTrue(run["eligible_for_profile"])
        self.assertEqual(metrics["road_ready_elapsed_ms"], 275)
        self.assertEqual(metrics["initial_startup_delay_ms"], 275)
        self.assertEqual(metrics["window_duration_ms"], 375)
        self.assertEqual(metrics["reliable_route_ms"], 375)
        self.assertAlmostEqual(metrics["reliable_abs_error_norm_milli_mean"], 420)

    def test_expired_zero_recovery_ramp_is_excluded_but_session_hold_wait_is_counted(self):
        entry, path = self._make_run("none", 1)
        reschedule_synthetic_fixture(path, [0, 50, 100, 150, 200, 250, 300, 400, 500])
        document = json.loads((path / "runtime-log.bin").read_text(encoding="utf-8"))
        rows = document["rows"]
        for index, row in enumerate(rows[:-1]):
            row.update(
                {
                    "mode": 10,
                    "vision_latest_valid": 1,
                    "vision_usable_fresh": 1,
                    "vision_timeout": 0,
                    "vision_hold_active": 0,
                    "vision_hold_expired": 0,
                    "motor_requested_permille": 500,
                    "servo_requested_us": 1530,
                    "servo_limited_us": 1520,
                    "servo_applied_us": 1520,
                    "motor_left_applied_permille": 500,
                    "motor_right_applied_permille": 500,
                    "tim3_ccr1_counts": 2400,
                    "tim3_ccr2_counts": 2375,
                    "vision_latest_error_norm_milli": 10,
                    "event_hold_enter": 0,
                    "event_hold_exit": 0,
                    "event_hold_deadline": 0,
                }
            )
        rows[2]["mode"] = 12
        rows[2]["vision_hold_active"] = 1
        rows[2]["event_hold_enter"] = 1
        rows[3]["mode"] = 12
        rows[3]["vision_hold_expired"] = 1
        rows[3]["event_hold_deadline"] = 1
        for index in (3, 4):
            rows[index]["motor_left_applied_permille"] = 0
            rows[index]["motor_right_applied_permille"] = 0
            rows[index]["servo_applied_us"] = 0
        rows[5]["motor_left_applied_permille"] = 200
        rows[5]["motor_right_applied_permille"] = 200
        rows[5]["servo_applied_us"] = 900
        rows[6]["motor_left_applied_permille"] = 350
        rows[6]["motor_right_applied_permille"] = 350
        rows[6]["servo_applied_us"] = 900
        rows[7]["motor_left_applied_permille"] = 550
        rows[7]["motor_right_applied_permille"] = 450
        rows[7]["servo_applied_us"] = 1520
        rows[-1]["mode"] = 0
        for field in analyzer.OUTPUT_ZERO_FIELDS:
            rows[-1][field] = 0
        header_stop = (document["header"]["start_ms"] + 500) & U32_MASK
        sync_synthetic_fixture(path, rows, header_updates={"stop_ms": header_stop})

        result = self._analyze([entry])
        run = result["runs"][0]
        metrics = run["metrics"]
        self.assertTrue(run["eligible_for_profile"])
        self.assertEqual(metrics["hold_active_ms"], 50)
        self.assertEqual(metrics["hold_expired_ms"], 50)
        self.assertEqual(metrics["hold_enter_event_count"], 1)
        self.assertEqual(metrics["hold_expired_event_count"], 1)
        self.assertEqual(metrics["recovery_wait_ms"], 100)
        self.assertEqual(metrics["recovery_ramp_ms"], 150)
        self.assertEqual(metrics["recovery_resume_count"], 1)
        self.assertEqual(metrics["servo_requested_vs_applied_abs_us_mean"], 10)
        self.assertEqual(metrics["servo_requested_vs_applied_abs_us_max"], 10)

    def test_servo_output_off_zero_does_not_count_as_a_reverse_command(self):
        entry, path = self._make_run("none", 1)
        document = json.loads((path / "runtime-log.bin").read_text(encoding="utf-8"))
        rows = document["rows"]
        rows[4]["servo_applied_us"] = 0
        sync_synthetic_fixture(path, rows)
        result = self._analyze([entry])
        metrics = result["runs"][0]["metrics"]
        self.assertEqual(metrics["servo_command_direction_reversals"], 0)

    def test_valid_hal_u32_wrap_is_accepted_and_inconsistent_wrap_is_rejected(self):
        start = 0xFFFFFF00
        valid, _ = self._make_run("none", 1, start_ms=start)
        valid_result = self._analyze([valid])
        self.assertTrue(valid_result["runs"][0]["eligible_for_profile"])

        bad, path = self._make_run("none", 2, start_ms=start)
        csv_path = path / "run.csv"
        with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
            fields = list(rows[0])
        rows[4]["t_ms"] = str((int(rows[4]["t_ms"]) + 1) & U32_MASK)
        with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        bad_result = self._analyze([bad])
        self.assertFalse(bad_result["runs"][0]["eligible_for_profile"])
        self.assertTrue(any("runtime-log.bin" in reason for reason in bad_result["runs"][0]["rejection_reasons"]))

        same_tick, path = self._make_run("none", 3)
        document = json.loads((path / "runtime-log.bin").read_text(encoding="utf-8"))
        rows = document["rows"]
        event_row = dict(rows[5])
        event_row["event_fault"] = 1
        event_row["fault"] = 3
        event_row["event_hold_deadline"] = 1
        event_row["vision_hold_active"] = 0
        event_row["vision_hold_expired"] = 1
        rows.insert(6, event_row)
        sync_synthetic_fixture(path, rows)
        same_tick_result = self._analyze([same_tick])
        same_tick_run = same_tick_result["runs"][0]
        self.assertTrue(same_tick_run["export_valid"])
        self.assertTrue(same_tick_run["eligible_for_profile"])
        self.assertEqual(same_tick_run["metrics"]["comm_fault_event_count"], 1)
        self.assertEqual(same_tick_run["metrics"]["hold_expired_event_count"], 1)
        self.assertEqual(same_tick_run["session_id"], 3)

        inconsistent_rows = [dict(row) for row in rows]
        direct_time_check = analyzer._validate_time(inconsistent_rows, 50, None)
        self.assertTrue(direct_time_check[0])
        inconsistent_rows[6]["t_elapsed_ms"] += 1
        direct_time_check = analyzer._validate_time(inconsistent_rows, 50, None)
        self.assertFalse(direct_time_check[0])
        self.assertIn("HAL u32 time wrap is inconsistent", direct_time_check[1][0])

    def test_corrupt_or_critical_missing_csv_fields_are_rejected(self):
        entry, path = self._make_run("none", 1)
        csv_path = path / "run.csv"
        with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
            fields = [field for field in rows[0] if field != "servo_applied_us"]
        with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows([{key: value for key, value in row.items() if key in fields} for row in rows])
        result = self._analyze([entry])
        self.assertFalse(result["runs"][0]["export_valid"])
        self.assertTrue(any("missing critical columns" in reason for reason in result["runs"][0]["rejection_reasons"]))

        entry, path = self._make_run("none", 2)
        csv_path = path / "run.csv"
        with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
            fields = list(rows[0])
        rows[3]["vision_latest_error_norm_milli"] = "broken"
        with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        result = self._analyze([entry])
        self.assertFalse(result["runs"][0]["export_valid"])
        self.assertTrue(any("non-integer vision_latest_error_norm_milli" in reason for reason in result["runs"][0]["rejection_reasons"]))

    def test_csv_numeric_tamper_against_raw_log_is_rejected(self):
        entry, path = self._make_run("none", 4)
        csv_path = path / "run.csv"
        with csv_path.open("r", encoding="utf-8-sig", newline="") as stream:
            rows = list(csv.DictReader(stream))
            fields = list(rows[0])
        rows[4]["vision_latest_error_norm_milli"] = "9999"
        with csv_path.open("w", encoding="utf-8-sig", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            writer.writerows(rows)
        result = self._analyze([entry])
        run = result["runs"][0]
        self.assertFalse(run["export_valid"])
        self.assertTrue(any("disagrees with decoded runtime-log.bin" in reason for reason in run["rejection_reasons"]))

    def test_duplicate_copied_session_cannot_count_twice(self):
        first, first_path = self._make_run("none", 1)
        duplicate, _ = self._make_run("none", 2)
        # Point the second manual entry at the first exported session.
        duplicate["path"] = str(first_path)
        duplicate["run_id"] = "operator-marked-second-but-same-frozen-data"
        third, _ = self._make_run("none", 3)
        result = self._analyze([first, duplicate, third])
        profile = next(item for item in result["profiles"] if item["variant"] == "none")
        self.assertEqual(profile["profile_eligible_runs"], 1)
        self.assertFalse(profile["eligible"])
        self.assertFalse(result["trial_set_complete"])
        self.assertTrue(any("duplicate frozen session copied" in reason for reason in result["runs"][1]["rejection_reasons"]))

    def test_each_profile_needs_three_observed_completed_in_track_sessions(self):
        runs = self._balanced_runs(completed=False)
        result = self._analyze(runs)
        self.assertEqual(result["analysis_status"], "synthetic-test-only")
        self.assertIsNone(result["recommendation"])
        self.assertTrue(all(item["profile_eligible_runs"] == 0 for item in result["profiles"]))
        self.assertTrue(all(item["synthetic_order"] is None for item in result["profiles"]))

    def test_different_layout_camera_lighting_or_voltage_blocks_global_ranking(self):
        cases = (
            ("layout_id", "layout-B", "layout_id differs"),
            ("camera_build_id", "camera-B", "camera_build_id differs"),
            ("lighting_id", "light-B", "lighting_id differs"),
            ("battery_voltage_v", 7.90, "battery voltage span"),
        )
        for field, value, expected_reason in cases:
            with self.subTest(field=field):
                runs = self._balanced_runs()
                runs[-1][field] = value
                result = self._analyze(runs)
                self.assertEqual(result["analysis_status"], "synthetic-test-only")
                self.assertFalse(result["comparison"]["layout_camera_lighting_battery_comparable"])
                self.assertIsNone(result["recommendation"])
                self.assertTrue(all(item["synthetic_order"] is None for item in result["profiles"]))
                self.assertTrue(any(expected_reason in reason for reason in result["comparison"]["condition_reasons"]))

        runs = self._balanced_runs()
        runs[-1]["battery_voltage_v"] = 7.8
        exact_boundary = self._analyze(runs)
        self.assertTrue(exact_boundary["comparison"]["layout_camera_lighting_battery_comparable"])
        self.assertAlmostEqual(exact_boundary["comparison"]["conditions"]["battery_voltage_span_v"], 0.2)

    def test_failed_dev_connect_export_is_diagnostic_only_and_not_a_trial(self):
        failed_dir = self.root / "failed-export"
        failed_dir.mkdir()
        failed_meta = {
            "status": "failed",
            "stage": "hotplug-header-initial",
            "error": "STM32CubeProgrammer DEV_CONNECT_ERR; no target RAM bytes",
            "initial_header": None,
            "raw_sha256": None,
            "raw_files": [],
        }
        (failed_dir / "run-metadata.json").write_text(json.dumps(failed_meta), encoding="utf-8")
        entry, _ = self._make_run("none", 1, metadata_mutate=lambda m, h: m.update(failed_meta))
        run_csv = Path(entry["path"]) / "run.csv"
        run_csv.unlink()
        result = self._analyze([entry], diagnostics_root=failed_dir)
        self.assertFalse(result["runs"][0]["export_valid"])
        self.assertFalse(result["runs"][0]["eligible_for_profile"])
        self.assertTrue(any("DEV_CONNECT_ERR" in reason for reason in result["runs"][0]["rejection_reasons"]))
        self.assertEqual(len(result["export_diagnostics"]), 1)
        self.assertEqual(result["export_diagnostics"][0]["error_code"], "DEV_CONNECT_ERR")
        self.assertFalse(result["export_diagnostics"][0]["csv_present"])

    def test_null_header_or_raw_hash_is_rejected_even_when_marked_success(self):
        entry, _ = self._make_run(
            "none", 1, metadata_mutate=lambda metadata, header: metadata.update(initial_header=None)
        )
        result = self._analyze([entry])
        self.assertFalse(result["runs"][0]["export_valid"])
        self.assertTrue(any("initial_header is null" in reason for reason in result["runs"][0]["rejection_reasons"]))

        entry, _ = self._make_run(
            "none", 2, metadata_mutate=lambda metadata, header: metadata.update(raw_sha256=None)
        )
        result = self._analyze([entry])
        self.assertFalse(result["runs"][0]["export_valid"])
        self.assertTrue(any("raw_sha256 is null" in reason for reason in result["runs"][0]["rejection_reasons"]))

    def test_wrong_elf_metadata_hash_and_truncated_full_runs_are_ineligible(self):
        wrong, _ = self._make_run(
            "none", 1, metadata_mutate=lambda metadata, header: metadata.update(elf_sha256="0" * 64)
        )
        result = self._analyze([wrong])
        self.assertFalse(result["runs"][0]["export_valid"])
        self.assertTrue(any("ELF profile/hash mismatch" in reason for reason in result["runs"][0]["rejection_reasons"]))

        def set_full(metadata, header):
            metadata["buffer_full"] = True
            header["buffer_full"] = True

        full, _ = self._make_run("none", 2, metadata_mutate=set_full)
        result = self._analyze([full])
        self.assertTrue(result["runs"][0]["export_valid"])
        self.assertFalse(result["runs"][0]["eligible_for_profile"])
        self.assertTrue(any("buffer_full" in reason for reason in result["runs"][0]["rejection_reasons"]))

        def set_truncated(metadata, header):
            metadata["records_truncated"] = True
            metadata["dropped_count"] = 1
            header["records_truncated"] = True
            header["dropped_count"] = 1

        truncated, _ = self._make_run("none", 3, metadata_mutate=set_truncated)
        result = self._analyze([truncated])
        self.assertTrue(result["runs"][0]["export_valid"])
        self.assertFalse(result["runs"][0]["eligible_for_profile"])
        self.assertTrue(any("records_truncated" in reason for reason in result["runs"][0]["rejection_reasons"]))

    def test_hardfault_and_mode8_exclude_but_comm_fault_is_reported_separately(self):
        def add_comm_fault(row, index, elapsed):
            if elapsed == 300:
                row["fault"] = 3
                row["event_fault"] = 1

        comm, _ = self._make_run("none", 1, mutate_row=add_comm_fault)
        result = self._analyze([comm])
        run = result["runs"][0]
        self.assertTrue(run["export_valid"])
        self.assertFalse(run["hard_fault"])
        self.assertTrue(run["eligible_for_profile"])
        self.assertEqual(run["metrics"]["comm_fault_event_count"], 1)
        self.assertGreater(run["metrics"]["comm_fault_ms"], 0)

        def add_hard_enum(row, index, elapsed):
            if elapsed == 300:
                row["fault"] = 1

        hard_enum, _ = self._make_run("none", 3, mutate_row=add_hard_enum)
        result = self._analyze([hard_enum])
        self.assertTrue(result["runs"][0]["hard_fault"])
        self.assertFalse(result["runs"][0]["eligible_for_profile"])

        def add_mode8(row, index, elapsed):
            if elapsed == 300:
                row["mode"] = 8
                row["fault"] = 3

        hard, _ = self._make_run("none", 2, mutate_row=add_mode8)
        result = self._analyze([hard])
        self.assertTrue(result["runs"][0]["hard_fault"])
        self.assertFalse(result["runs"][0]["eligible_for_profile"])
        self.assertTrue(any("mode=8" in reason for reason in result["runs"][0]["rejection_reasons"]))

        stop_fault, _ = self._make_run(
            "none", 4, metadata_mutate=lambda metadata, header: header.update(stop_reason=2)
        )
        result = self._analyze([stop_fault])
        self.assertTrue(result["runs"][0]["export_valid"])
        self.assertTrue(result["runs"][0]["hard_fault"])
        self.assertFalse(result["runs"][0]["eligible_for_profile"])

    def test_missing_simultaneous_motion_or_nonzero_final_record_is_rejected(self):
        def no_right_motor(row, index, elapsed):
            if elapsed < 500:
                row["motor_right_applied_permille"] = 0
                row["motor_left_applied_permille"] = 0
                row["tim3_ccr2_counts"] = 0
                row["tim3_ccr1_counts"] = 0

        no_motion, _ = self._make_run("none", 1, mutate_row=no_right_motor)
        result = self._analyze([no_motion])
        self.assertFalse(result["runs"][0]["eligible_for_profile"])
        self.assertTrue(any("road-ready sample" in reason for reason in result["runs"][0]["rejection_reasons"]))

        def final_nonzero(row, index, elapsed):
            if elapsed == 500:
                row["tim3_ccr4_counts"] = 1

        unsafe_stop, _ = self._make_run("none", 2, mutate_row=final_nonzero)
        result = self._analyze([unsafe_stop])
        self.assertFalse(result["runs"][0]["eligible_for_profile"])
        self.assertTrue(any("final frozen record" in reason for reason in result["runs"][0]["rejection_reasons"]))


if __name__ == "__main__":
    unittest.main()
