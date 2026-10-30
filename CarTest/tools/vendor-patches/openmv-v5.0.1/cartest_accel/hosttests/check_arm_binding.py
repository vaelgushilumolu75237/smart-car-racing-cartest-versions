"""Compile the optional binding/core with the local OpenMV H743 ARM toolchain.

Outputs are restricted to CarTest/logs/cartest-accel-prototype-20261026.
This compiles relocatable objects only; it does not rebuild or modify OpenMV.
"""

import hashlib
import json
import os
import subprocess
from pathlib import Path


TASK = Path(__file__).resolve().parents[1]
ROOT = TASK.parents[3]
OPENMV = ROOT / "tmp" / "openmv-v5.0.1-h750-audit"
GCC = Path(os.environ.get(
    "CARTEST_ARM_GCC",
    r"D:\OpenMV_IDE\share\qtcreator\arm\bin\arm-none-eabi-gcc.exe"))
OUTPUT_DIR = ROOT / "logs" / "cartest-accel-prototype-20261026" / "arm-binding"
QSTR_OVERLAY = OUTPUT_DIR / "include" / "genhdr" / "qstrdefs.generated.h"


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(65536), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    output_objects = OUTPUT_DIR / "objects"
    output_objects.mkdir(parents=True, exist_ok=True)
    (QSTR_OVERLAY.parent).mkdir(parents=True, exist_ok=True)

    generated_qstr = (
        OPENMV / "build" / "OPENMV4P" / "lib" / "micropython" /
        "genhdr" / "qstrdefs.generated.h")
    qstr_names = {
        "cartest_accel": "cartest_accel",
        "API_VERSION": "API_VERSION",
        "api_version": "api_version",
        "region_segments": "region_segments",
        "internal_component": "internal_component",
        "HEAP_STATS_API": "HEAP_STATS_API",
        "heap_stats": "heap_stats",
    }
    source_qstr = generated_qstr.read_text(encoding="utf-8")
    for name in qstr_names:
        if "MP_QSTR_" + name in source_qstr:
            raise RuntimeError("unexpected pre-existing static qstr: " + name)
    qstr_lines = [
        'QDEF0(MP_QSTR_%s, 0, %d, "%s")' % (name, len(value), value)
        for name, value in qstr_names.items()
    ]
    QSTR_OVERLAY.write_text(
        source_qstr + "\n/* Private API-1 syntax-check qstr overlay. */\n" +
        "\n".join(qstr_lines) + "\n", encoding="utf-8")

    include_dirs = [
        OUTPUT_DIR / "include",
        OPENMV / "lib" / "micropython",
        OPENMV / "lib" / "micropython" / "py",
        OPENMV / "lib" / "micropython" / "ports" / "stm32",
        OPENMV / "lib" / "micropython" / "ports" / "stm32" / "boards",
        OPENMV / "lib" / "micropython" / "ports" / "stm32" / "boards" / "OPENMV4P",
        OPENMV / "build" / "OPENMV4P" / "lib" / "micropython",
        OPENMV / "build" / "OPENMV4P" / "lib" / "micropython" / "genhdr",
        OPENMV / "lib" / "cmsis" / "include",
        OPENMV / "lib" / "cmsis" / "include" / "st",
        OPENMV / "lib" / "stm32" / "h7" / "include",
        OPENMV / "lib" / "tinyusb" / "src",
        OPENMV / "modules",
        TASK / "core",
    ]
    common_flags = [
        "-c", "-std=gnu99", "-Wall", "-Wpointer-arith", "-Werror",
        "-Wdouble-promotion", "-Wfloat-conversion",
        "-fno-inline-small-functions", "-Os",
        "-mcpu=cortex-m7", "-mthumb", "-mfpu=fpv5-d16", "-mfloat-abi=hard",
        "-DSTM32H743xx", "-DUSE_FULL_LL_DRIVER",
        "-DSTM32_HAL_H=<stm32h7xx_hal.h>",
    ] + ["-I" + str(path) for path in include_dirs]
    sources = [
        TASK / "binding" / "py_cartest_accel.c",
        TASK / "core" / "cartest_accel_core.c",
    ]
    version = subprocess.run(
        [str(GCC), "--version"], check=False, capture_output=True, text=True)
    records = []
    for source in sources:
        output = output_objects / (source.stem + ".o")
        command = [str(GCC)] + common_flags + ["-o", str(output), str(source)]
        result = subprocess.run(
            command, cwd=TASK, check=False, capture_output=True, text=True)
        record = {
            "source": str(source),
            "source_sha256": sha256(source),
            "object": str(output),
            "command": command,
            "exit_code": result.returncode,
            "stdout": result.stdout,
            "stderr": result.stderr,
        }
        if result.returncode == 0:
            record["object_sha256"] = sha256(output)
        records.append(record)

    report = {
        "toolchain": str(GCC),
        "toolchain_version": version.stdout,
        "target": "OpenMV v5.0.1 OPENMV4P / STM32H743 / Cortex-M7 hard-float",
        "openmv_tree": str(OPENMV),
        "generated_qstr_header": str(generated_qstr),
        "generated_qstr_header_sha256": sha256(generated_qstr),
        "qstr_overlay": str(QSTR_OVERLAY),
        "qstr_overlay_note": "Private syntax-check overlay for API-1 names; not a firmware build input.",
        "records": records,
        "scope": "Object compilation only; no full firmware link, flash, hardware, or performance claim.",
    }
    report_path = OUTPUT_DIR / "compile.json"
    report_path.write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(json.dumps(report, indent=2))
    if version.returncode != 0 or any(item["exit_code"] for item in records):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
