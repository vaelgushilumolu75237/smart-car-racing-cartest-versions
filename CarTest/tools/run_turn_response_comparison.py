"""Run a software-only opposite-target response comparison on production modules."""

from __future__ import annotations

import hashlib
import json
import re
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "test-data" / "turn-response-20261014" / "control"
HARNESS = ROOT / "tests" / "host" / "test_turn_response_timing.c"
SOURCE_FILES = [ROOT / "App" / "servo_safety.c", ROOT / "App" / "follow_output.c"]
PROFILES = [
    {
        "name": "legacy_2gear",
        "diff_permille": 100,
        "inner_diff_permille": 100,
        "normal_limit_permille": 600,
        "normal_motor_step_permille": 10,
        "normal_servo_step_us": 40,
        "left_target_permille": 400,
        "right_target_permille": 600,
    },
    {
        "name": "DebugFollowTurn_3gear",
        "diff_permille": 200,
        "inner_diff_permille": 200,
        "normal_limit_permille": 700,
        "normal_motor_step_permille": 20,
        "normal_servo_step_us": 60,
        "left_target_permille": 300,
        "right_target_permille": 700,
    },
]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def find_vsdevcmd() -> Path | None:
    candidates = [
        Path(r"D:\vs2026\Common7\Tools\VsDevCmd.bat"),
        Path(r"C:\Program Files\Microsoft Visual Studio\2022\Community\Common7\Tools\VsDevCmd.bat"),
        Path(r"C:\Program Files\Microsoft Visual Studio\2022\Professional\Common7\Tools\VsDevCmd.bat"),
        Path(r"C:\Program Files\Microsoft Visual Studio\2022\Enterprise\Common7\Tools\VsDevCmd.bat"),
        Path(r"C:\Program Files (x86)\Microsoft Visual Studio\2022\BuildTools\Common7\Tools\VsDevCmd.bat"),
    ]
    return next((path for path in candidates if path.is_file()), None)


def run_command(args: list[str], cwd: Path) -> subprocess.CompletedProcess[str]:
    compiler = shutil.which("cl")
    vsdevcmd = find_vsdevcmd()
    if vsdevcmd is not None:
        command = (f'call "{vsdevcmd}" -no_logo -arch=x64 >nul && '
                   + subprocess.list2cmdline([compiler or "cl", *args]))
        return subprocess.run(command, cwd=cwd, shell=True, capture_output=True,
                              text=True, errors="replace", check=False)
    if compiler:
        return subprocess.run([compiler, *args], cwd=cwd, capture_output=True,
                              text=True, errors="replace", check=False)
    pinned_compiler = Path(r"D:\vs2026\VC\Tools\MSVC\14.51.36231\bin\Hostx64\x64\cl.exe")
    if not pinned_compiler.is_file():
        raise RuntimeError("MSVC cl.exe and VsDevCmd.bat were not found")
    command = subprocess.list2cmdline([str(pinned_compiler), *args])
    return subprocess.run(command, cwd=cwd, shell=True, capture_output=True,
                          text=True, errors="replace", check=False)


def compile_profile(profile: dict[str, object]) -> dict[str, object]:
    name = str(profile["name"])
    executable = OUTPUT / f"turn-response-{name}.exe"
    compile_log = OUTPUT / f"turn-response-{name}-compile.log"
    run_log = OUTPUT / f"turn-response-{name}-run.log"
    compile_exit = OUTPUT / f"turn-response-{name}-compile.exitcode"
    run_exit = OUTPUT / f"turn-response-{name}-run.exitcode"
    args = [
        "/nologo", "/std:c11", "/W4", "/TC",
        "/I" + str(ROOT / "Inc"),
        f"/DCARTEST_FOLLOW_DIFF_PERMILLE={profile['diff_permille']}",
        f"/DCARTEST_FOLLOW_INNER_DIFF_PERMILLE={profile['inner_diff_permille']}",
        f"/DCARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE={profile['normal_limit_permille']}",
        "/DCARTEST_FOLLOW_HARD_LIMIT_PERMILLE=700",
        "/DCARTEST_FOLLOW_STARTUP_RAMP_STEP_PERMILLE=10",
        f"/DCARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE={profile['normal_motor_step_permille']}",
        f"/DCARTEST_FOLLOW_SERVO_SLEW_STEP_US={profile['normal_servo_step_us']}",
        "/Fe:" + str(executable), str(HARNESS),
        *(str(source) for source in SOURCE_FILES),
    ]
    built = run_command(args, OUTPUT)
    build_text = built.stdout + built.stderr
    compile_log.write_text(build_text, encoding="utf-8")
    compile_exit.write_text(f"{built.returncode}\n", encoding="ascii")
    if built.returncode != 0:
        run_log.write_text("NOT_RUN: MSVC compilation failed\n", encoding="utf-8")
        run_exit.write_text("-999\n", encoding="ascii")
        raise RuntimeError(f"{name} harness compile failed; see {compile_log}")
    ran = subprocess.run([str(executable)], cwd=OUTPUT, capture_output=True,
                         text=True, errors="replace", check=False)
    output = ran.stdout + ran.stderr
    run_log.write_text(output, encoding="utf-8")
    run_exit.write_text(f"{ran.returncode}\n", encoding="ascii")
    if ran.returncode != 0:
        raise RuntimeError(f"{name} timing harness failed; see {run_log}")
    match = re.search(r"^TURN_RESPONSE (.+)$", output, re.MULTILINE)
    if match is None:
        raise RuntimeError(f"{name} timing harness returned no result row")
    values: dict[str, object] = {"name": name}
    for item in match.group(1).split():
        key, value = item.split("=", 1)
        values[key] = int(value) if value.isdigit() else value
    values["compile_exit_code"] = built.returncode
    values["run_exit_code"] = ran.returncode
    values["compile_log"] = compile_log.name
    values["run_log"] = run_log.name
    return values


def main() -> int:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    results = [compile_profile(profile) for profile in PROFILES]
    by_name = {str(result["name"]): result for result in results}
    old = by_name["legacy_2gear"]
    new = by_name["DebugFollowTurn_3gear"]
    comparison = {
        "schema_version": 1,
        "evidence_type": "synthetic host simulation using production C modules",
        "generated_utc": datetime.now(timezone.utc).isoformat(),
        "mechanical_speed_measured": False,
        "scenario": {
            "description": "One full software servo target reversal from the right endpoint to the left endpoint.",
            "requested_servo_us": 1229,
            "starting_servo_limited_us": 2209,
            "follow_calibrated_left_endpoint_us": 1100,
            "follow_calibrated_center_us": 1530,
            "follow_calibrated_right_endpoint_us": 2500,
            "servo_slew_period_ms": 20,
            "motor_output_period_ms": 10,
            "steering_gain_permille": 700,
            "base_target_permille": 500,
        },
        "production_inputs": {
            str(path.relative_to(ROOT)): sha256(path)
            for path in [HARNESS, *SOURCE_FILES,
                         ROOT / "Inc" / "servo_safety.h",
                         ROOT / "Inc" / "follow_output.h"]
        },
        "profiles": results,
        "comparison": {
            "servo_endpoint_time_saved_ms": int(old["servo_endpoint_ms"]) - int(new["servo_endpoint_ms"]),
            "motor_endpoint_time_saved_ms": int(old["motor_endpoint_ms"]) - int(new["motor_endpoint_ms"]),
            "interpretation": "Software-request/limited/applied timing only; it does not estimate measured servo or vehicle mechanical speed.",
        },
    }
    json_path = OUTPUT / "turn-response-comparison.json"
    json_path.write_text(json.dumps(comparison, indent=2) + "\n", encoding="utf-8")
    md = [
        "# Synthetic opposite-target response comparison",
        "",
        "This host simulation compiles the production `App/servo_safety.c` and `App/follow_output.c` with the existing 70/30 steering gain and integer rounding. It starts with a software-applied right endpoint, requests the opposite endpoint, and advances servo and motor loops at their configured software periods.",
        "",
        "The measurements are software requested/limited/applied timing and per-side PWM command steps. They do not represent a measured servo speed, wheel speed, or vehicle response.",
        "",
        "| Profile | Full-turn target | Normal servo step | Servo reaches opposite endpoint | Normal motor step | Motor reaches opposite target | Max observed per-side software step |",
        "| --- | ---: | ---: | ---: | ---: | ---: | ---: |",
    ]
    for result in results:
        outer = int(result["profile_diff"])
        inner = int(result["inner_diff"])
        md.append(
            f"| {result['name']} | {500 + outer}/{500 - inner}‰ | "
            f"{result['servo_step_us']} µs / 20 ms | {result['servo_endpoint_ms']} ms | "
            f"{result['motor_step_permille']}‰ / 10 ms | {result['motor_endpoint_ms']} ms | "
            f"{result['max_actual_side_pwm_step_permille']}‰ |"
        )
    md.extend([
        "",
        f"The new profile reaches the software servo endpoint {comparison['comparison']['servo_endpoint_time_saved_ms']} ms sooner and the software motor target {comparison['comparison']['motor_endpoint_time_saved_ms']} ms sooner in this fixed synthetic reversal.",
        "",
        "The harness also checks a delayed loop interval: production code takes one software slew step and does not catch up multiple steps in one iteration.",
        "",
        "See `turn-response-comparison.json` for compile/run exit codes and source hashes. No board, servo, motor, or vehicle was connected.",
        "",
    ])
    (OUTPUT / "turn-response-comparison.md").write_text("\n".join(md), encoding="utf-8")
    print(json.dumps(comparison, indent=2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:  # surfaced with evidence paths for the build operator
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
