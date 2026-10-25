"""Save a source/ELF checkpoint and optionally publish it to the owner's GitHub.

Credentials are obtained from Git Credential Manager in memory and never logged.
The checkpoint worktree is separate from the live CarTest source directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import struct
import subprocess
import urllib.error
import urllib.request

PROJECT = Path(__file__).resolve().parents[1]
STORE = PROJECT / "deliverables" / "github-checkpoints" / "worktree"
REPO_NAME = "smart-car-racing-cartest-versions"
DIRS = ("App", "Inc", "Src", "Drivers", "cmake", "tools", "tests", "docs", ".vscode", "openmv")
ROOT_FILES = ("CMakeLists.txt", "CMakePresets.json", "CarTest.code-workspace",
              "project-description.json", "stm32h743xi_flash.ld", "README.md", "AGENTS.md",
              "README-恢复说明.md", "origin-manifest.json", "SHA256SUMS.txt")
SKIP_DIRS = {"__pycache__", ".git", "logs", "tmp", "node_modules"}
PUBLIC_COPY_EXCLUDED_SUBTREES = {
    # Experiment harnesses can refer to private logs; only this exact subtree
    # is kept out of public source checkpoints. Other directories named
    # "experiments" remain eligible for normal source copying.
    "tools": ("experiments",),
}
SKIP_SUFFIXES = {".pyc", ".obj", ".o", ".exe", ".dll", ".png", ".jpg",
                 ".jpeg", ".mp4", ".avi", ".bin", ".log"}
# Only these synthetic/source fixtures are public; never copy the vehicle-data
# tree wholesale. They are required by the shipped host regression commands.
PUBLIC_TEST_FIXTURES = (
    "test-data/OpenMV_V7_FAST_GRAY_RT45_20261005.manifest.json",
    "test-data/OpenMV_V7_FAST_GRAY_RT45_20261005.zip",
    "test-data/runtime-log-ab-20261012/baseline/Src__main.c",
    "deliverables/CarTest-20261014-0422-after/build/DebugFollowTurn/CarTest.elf",
)
PUBLIC_TEST_FIXTURE_SHA256 = {
    "deliverables/CarTest-20261014-0422-after/build/DebugFollowTurn/CarTest.elf":
        "67f88cafcb1e30281ab136a535d96f71ca698c71e52efb3fcbd545f3b18fcf48",
}
UPHILL_A_BASELINE_SHA256 = "667c95a7aef79b2630486faf7421a87e067ee582439dbcfad166d2203222718d"
UPHILL_FROZEN_PROFILE_SHA256 = {
    "B": "f50336905ea5a072f687a2a0edaa74412f115ed5e21f40ced85c0dd378e0866c",
    "C": "3c7da3c26054764bec0949753212272d37857c015d5f351d13b4a0b085755d4f",
}
# The public checkpoint carries only these explicit H743 profile artifacts,
# their summaries, and named pair/evidence documents. Do not copy the broader
# deliverables tree or build logs.
UPHILL_PROFILE_DELIVERIES = {
    "B": {
        "profile": "DebugFollowTurn65",
        "elf": "deliverables/H743_Uphill_20261019/B_65_35/CarTest.elf",
        "manifest": "deliverables/H743_Uphill_20261019/B_65_35/manifest.json",
    },
    "C": {
        "profile": "DebugFollowIncrementalPI",
        "elf": "deliverables/H743_Uphill_20261019/C_IncrementalPI/CarTest.elf",
        "manifest": "deliverables/H743_Uphill_20261019/C_IncrementalPI/manifest.json",
    },
}
UPHILL_ARTIFACT_MANIFEST = "deliverables/H743_Uphill_20261019/artifact-manifest.json"
UPHILL_BUILD_SUMMARY = "deliverables/H743_Uphill_20261019/build-summary.json"
SPEEDPD_PROFILE_DELIVERIES = {
    "B": {
        "profile": "DebugFollowTurn65",
        "elf": "deliverables/H743_SpeedPD_20261020/B_OpenLoop/CarTest.elf",
        "manifest": "deliverables/H743_SpeedPD_20261020/B_OpenLoop/manifest.json",
    },
    "C": {
        "profile": "DebugFollowIncrementalPI",
        "elf": "deliverables/H743_SpeedPD_20261020/C_IncrementalPI/CarTest.elf",
        "manifest": "deliverables/H743_SpeedPD_20261020/C_IncrementalPI/manifest.json",
    },
}
SPEEDPD_ARTIFACT_MANIFEST = "deliverables/H743_SpeedPD_20261020/artifact-manifest.json"
SPEEDPD_BUILD_SUMMARY = "deliverables/H743_SpeedPD_20261020/build-summary.json"
OPENMV_SPEEDPD_ROOT = "deliverables/OpenMV_Bottom80_Steering_20261021"
OPENMV_SPEEDPD_PACKAGES = ("no_pd", "pd", "pd_preview")
ACKERMANN_DELIVERY_ROOT = "deliverables/H743_Ackermann60_20261022"
ACKERMANN_PROFILE_DELIVERIES = {
    "coordinated": {
        "profile": "DebugFollowIncrementalPI",
        "directory": "C_Coordinated",
        "elf": f"{ACKERMANN_DELIVERY_ROOT}/C_Coordinated/CarTest.elf",
        "manifest": f"{ACKERMANN_DELIVERY_ROOT}/C_Coordinated/manifest.json",
        "coordination_enabled": True,
        "servo_slew_step_us": 160,
    },
    "baseline": {
        "profile": "DebugFollowIncrementalPIBaseline",
        "directory": "C_Baseline",
        "elf": f"{ACKERMANN_DELIVERY_ROOT}/C_Baseline/CarTest.elf",
        "manifest": f"{ACKERMANN_DELIVERY_ROOT}/C_Baseline/manifest.json",
        "coordination_enabled": False,
        "servo_slew_step_us": 120,
    },
}
ACKERMANN_ARTIFACT_MANIFEST = f"{ACKERMANN_DELIVERY_ROOT}/artifact-manifest.json"
ACKERMANN_BUILD_SUMMARY = f"{ACKERMANN_DELIVERY_ROOT}/build-summary.json"
ACKERMANN_PAIRED_DELIVERY_DOC = f"{ACKERMANN_DELIVERY_ROOT}/PAIRED_DELIVERY.md"
ACKERMANN_PAIRED_RUN_STEPS = f"{ACKERMANN_DELIVERY_ROOT}/validation/PAIRED_RUN_STEPS.md"
ACKERMANN_TOOLING_EVIDENCE = (
    f"{ACKERMANN_DELIVERY_ROOT}/validation/protocol-tooling/TEST_EVIDENCE.md")
OPENMV_ACKERMANN_ROOT = "deliverables/OpenMV_Ackermann320_20261022"
OPENMV_ACKERMANN_PACKAGES = ("geometry_perf_only", "coordinated")
OPENMV_ACKERMANN_RUNTIME_FILES = frozenset({
    "adaptive_detector.py", "boundary_evidence.py", "config.py",
    "contour_route.py", "contour_trace.py", "fast_boundary.py", "fast_gray.py",
    "fast_path.py", "fast_single_edge.py", "fast_topology.py",
    "grayscale_adapter.py", "line_detector.py", "main.py", "reverse_control.py",
    "road_detector.py", "source_selector.py", "surface_detector.py",
    "thresholding.py", "vision_protocol.py",
})
OPENMV_USB60_CANDIDATE_ROOT = "deliverables/OpenMV_USB60_Candidate_20261024"
OPENMV_USB60_CANDIDATE_PACKAGE_ID = "OPENMV_USB60_CANDIDATE_20261024"
OPENMV_USB60_CANDIDATE_RUNTIME_FILES = OPENMV_ACKERMANN_RUNTIME_FILES
OPENMV_USB60_CANDIDATE_DOCUMENTS = ("README.md", "CROSS_COMPILE_SUMMARY.json")
OPENMV_USB60_CANDIDATE_MANIFEST = "PACKAGE_MANIFEST.json"
OPENMV_USB60_BASELINE_COORDINATED_MANIFEST_SHA256 = (
    "49b9169425c39f5b7a453769ed41e8119063e7a5b0532260b8018f679dc6945d")
OPENMV_USB60_BASELINE_H743_ELFS = {
    ACKERMANN_PROFILE_DELIVERIES["coordinated"]["elf"]:
        "6370200c9b843cb1ecb5983b1eb886910a77eb145dff9f51cd119074bbd36c70",
    ACKERMANN_PROFILE_DELIVERIES["baseline"]["elf"]:
        "3729bd00e6015f06a106d3c0a54352e5f3dbd0f17a6d129a252bee59912b7655",
}
OPENMV_USB60_HW_VALIDATION_FALSE_FIELDS = frozenset({
    "camera_uploaded", "camera_board_execution_verified",
    "usb_preview_hardware_tested", "full_pipeline_60fps_verified",
    "h743_flashed_or_reset", "motor_started", "vehicle_tested",
})
OPENMV_USB60_CHECKPOINT_STATUS = "software-tested-hardware-pending"
OPENMV_NATIVE_PREVIEW_ROOT = "deliverables/OpenMV_NativePreview_20261024"
OPENMV_NATIVE_PREVIEW_REQUIRED_FILES = tuple(
    f"{OPENMV_NATIVE_PREVIEW_ROOT}/{name}" for name in (
        "firmware.bin", "firmware.elf", "build-summary.txt",
        "build-summary.txt.sha256", "artifact-manifest.json",
    )
)
# These two named review documents may be added by the release owner. No other
# candidate-folder files (including the CI zip and logs) enter the checkpoint.
OPENMV_NATIVE_PREVIEW_OPTIONAL_DOCUMENTS = tuple(
    f"{OPENMV_NATIVE_PREVIEW_ROOT}/{name}" for name in ("README.md", "EVIDENCE.md")
)
OPENMV_NATIVE_PREVIEW_PATCH = (
    "tools/vendor-patches/openmv-v5.0.0/0001-csi-snapshot-preview-kwarg.patch")
OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256 = (
    "2bbb92b2b25705002ae16b9e9f18f57fc171ca8f0549680cc15260cce0b468d5")
OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256 = {
    "build-summary.txt": "5f448297965d25ec76807ca3e3404ca4753c3acf73880c9dd5cf2bae87996dd7",
    "build-summary.txt.sha256": "4b7467ab38a26d14ffc08c7b94cc6f90465ba8133ff524d2487d63e2f7343876",
    "firmware.bin": "9689d686881d2d358694f580256d0698f01226cd018c6f6951d1d7abf5c663b7",
    "firmware.elf": "dafaec93152163769e81e0f86fcd9b280b289f4e6a49a87d1c29ac1fd559caaa",
}
OPENMV_NATIVE_PREVIEW_EXPECTED_BUILD = {
    "openmv_tag": "v5.0.0",
    "openmv_commit": "7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79",
    "openmv_sdk_version": "1.6.0",
    "patch_sha256": OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256,
    "firmware.bin_sha256": OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256["firmware.bin"],
    "firmware.elf_sha256": OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256["firmware.elf"],
}
OPENMV_NATIVE_PREVIEW_RUN_ID = 40003611704
OPENMV_NATIVE_PREVIEW_BIN_BYTES = 1_736_532
OPENMV_NATIVE_PREVIEW_ELF_BYTES = 11_322_036
OPENMV_NATIVE_PREVIEW_VECTOR_SP = 0x00010000
OPENMV_NATIVE_PREVIEW_VECTOR_RESET = 0x08040299
OPENMV_UART_FIFO_FIRMWARE_ROOT = "deliverables/OpenMV_NativePreview_UARTFIFO_20261024"
OPENMV_UART_FIFO_FIRMWARE_FILES = tuple(
    f"{OPENMV_UART_FIFO_FIRMWARE_ROOT}/{name}" for name in (
        "firmware.bin", "firmware.elf", "build-summary.txt",
        "build-summary.txt.sha256", "artifact-manifest.json", "README.md",
        "EVIDENCE.md",
    )
)
OPENMV_UART_FIFO_FIRMWARE_BUILD_FILES = {
    "firmware.bin": {
        "bytes": 1_736_932,
        "sha256": "0bbf81a534f99365a62187b3116559e313527e0aa1b7d585e618022b11feffc3",
    },
    "firmware.elf": {
        "bytes": 11_330_088,
        "sha256": "aae890556a1f35fe784ec451c064e4e24d1710ef91cff9f3f051117f1c18cde6",
    },
    "build-summary.txt": {
        "bytes": 556,
        "sha256": "3f18790bf7666a44326e80319abfda115654befc1618445a044ecc0c55299763",
    },
    "build-summary.txt.sha256": {
        "bytes": 84,
        "sha256": "80c0b3252a3bcaa24552e9992138d504bd2ddbbbff2b39c07fad232ce4d9c23a",
    },
}
OPENMV_UART_FIFO_FIRMWARE_REVIEW_DOCS_SHA256 = {
    "README.md": "0e67dd0552e200c1a1518d656d632407bdadd1d17870baee029e9f797c94d2e3",
    "EVIDENCE.md": "c1e8b3d4ca51e594a926690e018e761a702de758b67e486ed406c3f22ea9f720",
}
OPENMV_UART_FIFO_PATCH = (
    "tools/vendor-patches/openmv-v5.0.0/0002-stm32h7-uart-fifo-write.patch")
OPENMV_UART_FIFO_PATCH_MANIFEST = (
    "tools/vendor-patches/openmv-v5.0.0/0002-stm32h7-uart-fifo-write.manifest.json")
OPENMV_UART_FIFO_PATCH_README = (
    "tools/vendor-patches/openmv-v5.0.0/0002-stm32h7-uart-fifo-write.README.md")
OPENMV_UART_FIFO_PATCH_SHA256 = (
    "8d1649cc9fb74e00b6d1234f6c8a3a62d70b90ebd605053f069b202ec36890fb")
OPENMV_UART_FIFO_PATCH_MANIFEST_SHA256 = (
    "b775d9214cea40c2e3cc2beb1f04fae372ed2f57352b25fb64adfba4a1897179")
OPENMV_UART_FIFO_PATCH_README_SHA256 = (
    "d4c90a53f51145f4672753466ca0661a0fe0a736c964f97b0f4286deafc3a717")
OPENMV_UART_FIFO_PATCH_MANIFEST_EXPECTED = {
    "schema_version": 1,
    "patch_file": "0002-stm32h7-uart-fifo-write.patch",
    "patch_sha256": OPENMV_UART_FIFO_PATCH_SHA256,
    "series_position": 2,
    "applies_after": {
        "patch_file": "0001-csi-snapshot-preview-kwarg.patch",
        "patch_sha256": OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256,
    },
    "base": {
        "repository": "https://github.com/openmv/openmv",
        "tag": "v5.0.0",
        "openmv_commit": "7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79",
        "micropython_commit": "d18a77e5d55664045e71a0c839695df07a96a6e1",
    },
    "base_files_sha256": {
        "lib/micropython/extmod/machine_uart.c":
            "ea18b5142a4f19183dcb7583781f78e5f41ccb44938fcd32c849c61fe43a85a3",
        "lib/micropython/ports/stm32/machine_uart.c":
            "48e0522634fed71eeea4a5e94e70ebd39fd4e4e8fff8630e87758f9f7644fea0",
        "lib/micropython/ports/stm32/uart.c":
            "dd6de2dc5556ad5d66c48a95232e8c565a4dea32c60d8e348e3273c07d4b157d",
        "lib/micropython/ports/stm32/uart.h":
            "9992a97d1ec5e9868d47b7ea9a37af4f44040230f04ddf8d7fe9e4c42dfcfbf0",
    },
    "api": {
        "marker": "UART.WRITE_FIFO_API == 1 on STM32H7 only",
        "write_fifo_max_bytes": 16,
        "ordinary_write_unchanged": True,
    },
}
OPENMV_UART_FIFO_FIRMWARE_BUILD_SUMMARY_EXPECTED = {
    "openmv_tag": "v5.0.0",
    "openmv_commit": "7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79",
    "openmv_sdk_version": "1.6.0",
    "micropython_commit": "d18a77e5d55664045e71a0c839695df07a96a6e1",
    "patch_0001_preview_sha256": OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256,
    "patch_0002_uart_fifo_sha256": OPENMV_UART_FIFO_PATCH_SHA256,
    "preview_control_api": "1",
    "uart_write_fifo_api": "1",
    "firmware.bin_sha256": OPENMV_UART_FIFO_FIRMWARE_BUILD_FILES["firmware.bin"]["sha256"],
    "firmware.elf_sha256": OPENMV_UART_FIFO_FIRMWARE_BUILD_FILES["firmware.elf"]["sha256"],
}
OPENMV_UART_FIFO_CI_RUN_ID = 40095756045
OPENMV_UART_FIFO_CI_SOURCE_COMMIT = "a44ba5186711e37cde4a65c19b6b3dd9780e75af"
OPENMV_UART_FIFO_EXPECTED_ARTIFACT_MANIFEST = {
    "schema_version": 1,
    "package_id": "OpenMV_NativePreview_UARTFIFO_20261024",
    "target": {"board": "OPENMV4P", "mcu": "STM32H743"},
    "upstream": {
        "repository": "https://github.com/openmv/openmv",
        "tag": "v5.0.0",
        "openmv_commit": "7b7b4e73cb6a0fd09d793201a5c3c588ca0ffa79",
        "micropython_commit": "d18a77e5d55664045e71a0c839695df07a96a6e1",
    },
    "patches": {
        "0001-csi-snapshot-preview-kwarg.patch":
            OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256,
        "0002-stm32h7-uart-fifo-write.patch": OPENMV_UART_FIFO_PATCH_SHA256,
    },
    "build": {
        "provider": "GitHub Actions",
        "repository": "vaelgushilumolu75237/smart-car-racing-cartest-versions",
        "run_id": OPENMV_UART_FIFO_CI_RUN_ID,
        "source_commit": OPENMV_UART_FIFO_CI_SOURCE_COMMIT,
        "conclusion": "success",
        "openmv_sdk_version": "1.6.0",
    },
    "api_markers": {"preview_control_api": 1, "uart_write_fifo_api": 1},
    "files_sha256": OPENMV_UART_FIFO_FIRMWARE_BUILD_FILES,
    "review_documents_sha256": OPENMV_UART_FIFO_FIRMWARE_REVIEW_DOCS_SHA256,
    "reported_hardware_evidence": {
        "flash_readback": {
            "status": "reported_full_image_hash_match",
            "date": "2026-10-25",
            "sha256": OPENMV_UART_FIFO_FIRMWARE_BUILD_FILES["firmware.bin"]["sha256"],
        },
        "static_uart_fifo_invalid_only": {
            "status": "static_invalid_only",
            "preview_wrapper_seconds": 65,
            "board_window_fps_range": [60.6, 60.7],
            "host_marker_window_fps_range": [60.2, 60.8],
            "complete_board_window_count": 6,
            "complete_host_marker_window_count": 6,
            "fifo_accepted_frames": 3944,
            "fifo_accepted_bytes": 63104,
            "partial_writes": 0,
            "busy_results": 0,
            "errors": 0,
            "tx_complete": True,
            "h743_read_only_rx_seconds": 1,
            "invalid_frames": 60,
            "valid_route_frames": 0,
            "sequence_baseline": 0,
            "rejected_sequence": 60,
            "control_outputs": 0,
            "tokens": 0,
            "events": 0,
        },
        "newer_ide_test": {
            "status": "static_payload_read_measured",
            "duration_seconds": 65,
            "module_batch_size": 19,
            "uart_fifo_wrapper": "same_as_static_uart_fifo_run",
            "wire_mode": "intentionally_invalid_fixed_sequence_zero",
            "board_performance_windows": {
                "window_seconds": 10,
                "complete_window_count": 6,
                "fps_samples": [60.7, 60.8, 60.7, 60.8, 60.6, 60.8],
                "fps_range": [60.6, 60.8],
                "valid_in_all_windows": True,
                "uart_errors": 0,
            },
            "fifo": {
                "accepted_frames": 3949,
                "accepted_bytes": 63184,
                "busy_results": 0,
                "short_writes": 0,
                "errors": 0,
                "tx_complete_before": True,
                "tx_complete_after": True,
            },
            "ide_payload_read": {
                "complete_window_count": 6,
                "rate_method": (
                    "delta_FrameDataSize_records/"
                    "(current_read_end-previous_read_begin)"),
                "rate_hz_samples": [
                    60.5142647373, 60.2316303329, 60.2352603627,
                    60.1152914784, 60.3861933008, 60.1489975176,
                ],
                "rate_hz_range": [60.1152914784, 60.5142647373],
                "advertised_size_records": 3986,
                "complete_payload_read_records": 3986,
                "advertised_and_read_sizes_all_equal": True,
                "error_lines": 0,
                "unique_frame_identity_verified": False,
            },
            "limitations": [
                "IDE data establishes complete payload reads only; independent per-frame JPEG decode was not measured.",
                "IDE marker freshness and physical display refresh were not measured.",
                "Wire frames remained intentionally invalid with sequence zero; valid route control was not tested.",
                "The separate SDK marker result is not merged with IDE payload-read measurements.",
            ],
        },
        "separate_sdk_marker_measurement": {
            "status": "measured_in_separate_run",
            "complete_window_count": 6,
            "fps_range": [60.2, 60.8],
            "supports_independent_decoding_of_new_frames": True,
            "proves_ide_per_frame_decode_or_physical_refresh": False,
        },
        "new_usb_headless_test": {
            "status": "static_fifo_invalid_only_cdc_closed_measured",
            "duration_seconds": 65,
            "capture_scope": "real_sensor",
            "camera_main_sha256": (
                "044416e9161ba824bc9037ed7f40260003bfe942905d02106721a41b6cc26b06"),
            "same_camera_main_as_ide_payload_read_test": True,
            "wire_mode": "intentionally_invalid_fixed_sequence_zero",
            "cdc_close_seconds": 68.0,
            "report_physical_usb_cable_state": "not_measured",
            "lead_observed_physical_usb_cable_state": "connected",
            "physical_usb_cable_removed": False,
            "headless_report": {
                "preview_state": "NATIVE_HEADLESS",
                "cdc_close_confirmed": True,
                "headless_observed": True,
                "jpeg_marker_writes": 0,
                "preview_errors": 0,
                "buffer_overflow": 0,
            },
            "board_performance_windows": {
                "window_seconds": 10,
                "complete_window_count": 6,
                "capture_samples": [794, 794, 792, 794, 792, 793],
                "valid_samples": [794, 794, 792, 794, 792, 793],
                "accepted_samples": [794, 794, 792, 794, 792, 793],
                "fps_samples": [79.4, 79.4, 79.2, 79.4, 79.2, 79.3],
                "fps_range": [79.2, 79.4],
                "write_errors": 0,
                "total_cycle_p95_upper_us_samples": [
                    13000, 13000, 13000, 13000, 13000, 13000,
                ],
                "total_cycle_max_us": 30759,
            },
            "fifo": {
                "attempts": 5156,
                "accepted_frames": 5156,
                "accepted_bytes": 82496,
                "busy_results": 0,
                "partial_writes": 0,
                "errors": 0,
                "tx_complete_before": True,
                "tx_complete_after": True,
                "wire_sequence": 0,
                "wire_valid": False,
                "computed_route_forwarded": 0,
            },
            "separate_read_only_swd_observation": {
                "duration_seconds": 1,
                "frames": 79,
                "invalid_frames": 79,
                "wire_sequence_value": 0,
                "rejected_fixed_sequence_frames": 79,
                "parser_errors": 0,
                "valid_route_frames": 0,
                "outputs": 0,
                "tokens": 0,
                "events": 0,
            },
            "headless_report_delivery": {
                "complete_report_bytes": 2271,
                "line_return_succeeded": True,
                "prior_failed_attempt_records_retained": True,
            },
            "limitations": [
                "CDC was closed for 68 seconds but the physical USB cable remained inserted; this is not a physical disconnect test.",
                "Wire frames were intentionally invalid with fixed sequence zero; the separate SWD observation rejected invalid frames and did not validate route control.",
                "This measurement does not establish effective route control, road behavior, or full end-to-end acceptance.",
                "IDE payload-read and separate SDK marker measurements remain separate from this headless run.",
            ],
        },
    },
    "pending_acceptance": {
        "physical_usb_disconnect_test": "pending",
        "valid_route_control": "pending",
        "end_to_end_60fps_acceptance": "pending",
        "road_vehicle_acceptance": "pending",
    },
}
OPENMV_UART_FIFO_BIN_BYTES = 1_736_932
OPENMV_UART_FIFO_ELF_BYTES = 11_330_088
OPENMV_UART_FIFO_VECTOR_SP = 0x00010000
OPENMV_UART_FIFO_VECTOR_RESET = 0x08040299
PUBLIC_DELIVERY_FILES = (
    "deliverables/H743_Uphill_20261019/A_Baseline/CarTest.elf",
    "deliverables/H743_Uphill_20261019/A_Baseline/manifest.json",
    UPHILL_PROFILE_DELIVERIES["B"]["elf"],
    UPHILL_PROFILE_DELIVERIES["B"]["manifest"],
    UPHILL_PROFILE_DELIVERIES["C"]["elf"],
    UPHILL_PROFILE_DELIVERIES["C"]["manifest"],
    UPHILL_ARTIFACT_MANIFEST,
    UPHILL_BUILD_SUMMARY,
    SPEEDPD_PROFILE_DELIVERIES["B"]["elf"],
    SPEEDPD_PROFILE_DELIVERIES["B"]["manifest"],
    SPEEDPD_PROFILE_DELIVERIES["C"]["elf"],
    SPEEDPD_PROFILE_DELIVERIES["C"]["manifest"],
    SPEEDPD_ARTIFACT_MANIFEST,
    SPEEDPD_BUILD_SUMMARY,
    ACKERMANN_PROFILE_DELIVERIES["coordinated"]["elf"],
    ACKERMANN_PROFILE_DELIVERIES["coordinated"]["manifest"],
    ACKERMANN_PROFILE_DELIVERIES["baseline"]["elf"],
    ACKERMANN_PROFILE_DELIVERIES["baseline"]["manifest"],
    ACKERMANN_ARTIFACT_MANIFEST,
    ACKERMANN_BUILD_SUMMARY,
    ACKERMANN_PAIRED_DELIVERY_DOC,
    ACKERMANN_PAIRED_RUN_STEPS,
    ACKERMANN_TOOLING_EVIDENCE,
)


def run_git(*args: str) -> str:
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never")
    result = subprocess.run(["git", "-C", str(STORE), *args], capture_output=True,
                            text=True, encoding="utf-8", env=env, timeout=180)
    if result.returncode:
        raise RuntimeError(f"git {args[0]} failed ({result.returncode}): {result.stderr.strip()}")
    return result.stdout.strip()


def api(path: str, token: str, body=None):
    headers = {"Authorization": "Bearer " + token,
               "User-Agent": "CarTest-version-checkpoint",
               "Accept": "application/vnd.github+json"}
    request = urllib.request.Request("https://api.github.com" + path, headers=headers,
                                    data=None if body is None else json.dumps(body).encode())
    try:
        with urllib.request.urlopen(request, timeout=30) as response:
            return json.load(response)
    except urllib.error.HTTPError as error:
        if error.code == 404 and body is None:
            return None
        raise RuntimeError(f"GitHub API {path}: HTTP {error.code}") from None


def authenticated_user():
    env = dict(os.environ, GIT_TERMINAL_PROMPT="0", GCM_INTERACTIVE="never")
    result = subprocess.run(["git", "credential", "fill"],
                            input="protocol=https\nhost=github.com\n\n", capture_output=True,
                            text=True, env=env, timeout=30)
    if result.returncode:
        raise RuntimeError("Git Credential Manager has no usable noninteractive GitHub credential.")
    credential = dict(line.split("=", 1) for line in result.stdout.splitlines() if "=" in line)
    token = credential.get("password", "")
    user = api("/user", token)
    if not user or not user.get("login"):
        raise RuntimeError("GitHub account verification failed.")
    return user, token


def safe_remove(path: Path):
    resolved = path.resolve()
    if resolved.parent != STORE.resolve() or resolved.name != "CarTest":
        raise RuntimeError("Refusing to clear a directory outside the checkpoint worktree.")
    if resolved.exists():
        shutil.rmtree(resolved)


def copy_tree(source: Path, destination: Path, *, excluded_subtrees=()):
    excluded = tuple(Path(item) for item in excluded_subtrees)
    for item in sorted(source.rglob("*")):
        relative = item.relative_to(source)
        if any(relative == subtree or subtree in relative.parents
               for subtree in excluded):
            continue
        if any(part in SKIP_DIRS for part in relative.parts):
            continue
        if item.is_symlink():
            raise RuntimeError(f"Symlink is not allowed in a public source checkpoint: {relative}")
        if item.is_file() and item.suffix.lower() not in SKIP_SUFFIXES:
            target = destination / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(item, target)


def copy_allowlisted_file(source: Path, destination: Path, relative_name: str):
    item = source / relative_name
    if not item.is_file() or item.is_symlink():
        raise RuntimeError(f"Required public delivery file is missing or linked: {relative_name}")
    target = destination / relative_name
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(item, target)


def normalize_relative_path(value) -> str | None:
    if not isinstance(value, str):
        return None
    return value.replace("\\", "/")


def index_build_rows(summary: dict) -> dict:
    builds = summary.get("builds", [])
    if isinstance(builds, dict):
        return {str(name): row for name, row in builds.items() if isinstance(row, dict)}
    if isinstance(builds, list):
        return {row.get("preset"): row for row in builds if isinstance(row, dict)}
    raise RuntimeError("Build summary must contain build rows keyed by preset or a list of rows.")


def verify_uphill_deliveries(source: Path):
    a_elf = source / PUBLIC_DELIVERY_FILES[0]
    a_manifest = source / PUBLIC_DELIVERY_FILES[1]
    if not a_elf.is_file() or a_elf.is_symlink():
        raise RuntimeError("The frozen uphill A ELF is missing or linked.")
    if hashlib.sha256(a_elf.read_bytes()).hexdigest() != UPHILL_A_BASELINE_SHA256:
        raise RuntimeError("Frozen uphill A ELF hash mismatch.")
    try:
        parsed_a = json.loads(a_manifest.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("Frozen uphill A manifest is missing or invalid.") from error
    if parsed_a.get("sha256") != UPHILL_A_BASELINE_SHA256:
        raise RuntimeError("Frozen uphill A manifest does not match the allowlisted ELF hash.")

    try:
        artifact_manifest = json.loads(
            (source / UPHILL_ARTIFACT_MANIFEST).read_text(encoding="utf-8-sig"))
        build_summary = json.loads(
            (source / UPHILL_BUILD_SUMMARY).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("Uphill artifact manifest or build summary is missing or invalid.") from error

    artifacts = artifact_manifest.get("artifacts", {})
    build_rows = index_build_rows(build_summary)
    for variant, spec in UPHILL_PROFILE_DELIVERIES.items():
        delivery_elf = source / spec["elf"]
        delivery_manifest_path = source / spec["manifest"]
        for path, label in ((delivery_elf, "delivery ELF"),
                            (delivery_manifest_path, "delivery manifest")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"Uphill {variant} {label} is missing or linked: {path}")

        delivery_bytes = delivery_elf.read_bytes()
        elf_sha256 = hashlib.sha256(delivery_bytes).hexdigest()
        if elf_sha256 != UPHILL_FROZEN_PROFILE_SHA256[variant]:
            raise RuntimeError(f"Frozen uphill {variant} delivery ELF hash mismatch.")
        try:
            variant_manifest = json.loads(
                delivery_manifest_path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"Uphill {variant} manifest is invalid.") from error
        if (variant_manifest.get("profile") != spec["profile"] or
                variant_manifest.get("sha256") != elf_sha256):
            raise RuntimeError(f"Uphill {variant} manifest does not match its ELF/profile.")

        artifact = artifacts.get(variant, {})
        if (normalize_relative_path(artifact.get("delivery_path")) != spec["elf"] or
                artifact.get("sha256") != elf_sha256 or
                normalize_relative_path(artifact.get("export_elf")) != spec["elf"]):
            raise RuntimeError(f"Top-level artifact manifest does not match uphill {variant}.")
        build_row = build_rows.get(spec["profile"], {})
        if (build_row.get("exit_code") != 0 or
                build_row.get("sha256") != elf_sha256 or
                normalize_relative_path(build_row.get("elf")) != f"build/{spec['profile']}/CarTest.elf"):
            raise RuntimeError(f"Frozen build summary does not match uphill {variant} ELF.")


def verify_speedpd_deliveries(source: Path):
    """Check the frozen SpeedPD pair against its own manifests and summary.

    The profile names are reused by later builds, so the current build output
    at ``build/<profile>`` is not authoritative for this historical delivery.
    """
    try:
        artifact_manifest = json.loads(
            (source / SPEEDPD_ARTIFACT_MANIFEST).read_text(encoding="utf-8-sig"))
        build_summary = json.loads(
            (source / SPEEDPD_BUILD_SUMMARY).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("SpeedPD artifact manifest or build summary is missing or invalid.") from error

    if (artifact_manifest.get("schema_version") != 1 or
            artifact_manifest.get("package") != "H743_SpeedPD_20261020" or
            normalize_relative_path(artifact_manifest.get("build_summary")) !=
            SPEEDPD_BUILD_SUMMARY):
        raise RuntimeError("SpeedPD artifact manifest metadata is inconsistent.")
    artifacts = artifact_manifest.get("artifacts", {})
    summary_artifacts = build_summary.get("artifacts", {})
    variant_manifests = artifact_manifest.get("per_variant_manifests", {})
    if build_summary.get("source_inputs_stable") is not True:
        raise RuntimeError("SpeedPD build summary does not certify stable source inputs.")
    if (build_summary.get("schema_version") != 1 or
            build_summary.get("package") != "H743_SpeedPD_20261020" or
            not isinstance(build_summary.get("source_sha256"), dict)):
        raise RuntimeError("SpeedPD build summary metadata is inconsistent.")
    build_rows = index_build_rows(build_summary)
    for variant, spec in SPEEDPD_PROFILE_DELIVERIES.items():
        delivery_elf = source / spec["elf"]
        delivery_manifest_path = source / spec["manifest"]
        for path, label in ((delivery_elf, "delivery ELF"),
                            (delivery_manifest_path, "delivery manifest")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"SpeedPD {variant} {label} is missing or linked: {path}")

        delivery_bytes = delivery_elf.read_bytes()
        elf_sha256 = hashlib.sha256(delivery_bytes).hexdigest()
        elf_size = len(delivery_bytes)
        try:
            variant_manifest = json.loads(
                delivery_manifest_path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"SpeedPD {variant} manifest is invalid.") from error
        if (variant_manifest.get("profile") != spec["profile"] or
                variant_manifest.get("sha256") != elf_sha256 or
                variant_manifest.get("size_bytes") != elf_size or
                variant_manifest.get("build_id") is None or
                normalize_relative_path(variant_manifest.get("delivery_path")) !=
                spec["elf"] or
                normalize_relative_path(variant_manifest.get("export_elf")) !=
                spec["elf"] or
                variant_manifest.get("source_sha256") !=
                build_summary["source_sha256"]):
            raise RuntimeError(f"SpeedPD {variant} manifest does not match its ELF/profile.")

        artifact = artifacts.get(variant, {})
        if (normalize_relative_path(artifact.get("delivery_path")) != spec["elf"] or
                artifact.get("sha256") != elf_sha256 or
                normalize_relative_path(artifact.get("export_elf")) != spec["elf"] or
                artifact.get("profile") != spec["profile"] or
                artifact.get("build_id") != variant_manifest.get("build_id")):
            raise RuntimeError(f"SpeedPD artifact manifest does not match {variant}.")
        summary_artifact = summary_artifacts.get(variant, {})
        if (normalize_relative_path(summary_artifact.get("delivery_path")) !=
                spec["elf"] or
                normalize_relative_path(summary_artifact.get("export_elf")) !=
                spec["elf"] or
                summary_artifact.get("sha256") != elf_sha256 or
                summary_artifact.get("profile") != spec["profile"] or
                summary_artifact.get("build_id") != variant_manifest.get("build_id")):
            raise RuntimeError(f"SpeedPD build summary artifact does not match {variant}.")
        if normalize_relative_path(variant_manifests.get(variant)) != spec["manifest"]:
            raise RuntimeError(f"SpeedPD artifact manifest does not name {variant} manifest.")

        build_row = build_rows.get(spec["profile"], {})
        if (build_row.get("exit_code") != 0 or
                build_row.get("sha256") != elf_sha256 or
                build_row.get("size_bytes") != elf_size or
                build_row.get("profile") != spec["profile"] or
                build_row.get("build_id") != variant_manifest.get("build_id") or
                normalize_relative_path(build_row.get("elf")) !=
                f"build/{spec['profile']}/CarTest.elf"):
            raise RuntimeError(f"Frozen SpeedPD build summary does not match {variant} ELF.")


def verify_ackermann_deliveries(source: Path, expected_presets: set[str]):
    """Check the exact coordinated/baseline ELF pair and bounded build evidence."""
    try:
        artifact_manifest = json.loads(
            (source / ACKERMANN_ARTIFACT_MANIFEST).read_text(encoding="utf-8-sig"))
        build_summary = json.loads(
            (source / ACKERMANN_BUILD_SUMMARY).read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("Ackermann artifact manifest or build summary is missing or invalid.") from error

    if (artifact_manifest.get("schema_version") != 1 or
            not artifact_manifest.get("generated_utc") or
            not artifact_manifest.get("package_id")):
        raise RuntimeError("Ackermann artifact manifest metadata is incomplete.")
    if (build_summary.get("schema_version") not in (None, 1) or
            build_summary.get("package_id", artifact_manifest["package_id"]) !=
            artifact_manifest["package_id"]):
        raise RuntimeError("Ackermann build summary package identity does not match its artifact manifest.")
    if build_summary.get("source_inputs_stable") is False:
        raise RuntimeError("Ackermann build summary reports that source inputs changed during build.")

    build_rows = index_build_rows(build_summary)
    if set(build_rows) != expected_presets:
        raise RuntimeError("Ackermann build summary must contain exactly the 12 declared build presets.")
    for preset, row in build_rows.items():
        row_path = normalize_relative_path(row.get("elf_path", row.get("elf")))
        row_bytes = row.get("elf_bytes", row.get("bytes"))
        row_sha = row.get("elf_sha256", row.get("sha256"))
        if (row.get("exit_code") != 0 or not row.get("log") or
                not row_path or row_bytes is None or not row_sha):
            raise RuntimeError(f"Ackermann build evidence is incomplete or failed for {preset}.")
        expected_path = f"build/{preset}/CarTest.elf"
        if row_path != expected_path:
            raise RuntimeError(f"Ackermann build summary names an unexpected ELF path for {preset}.")
        build_elf = source / expected_path
        log_value = normalize_relative_path(row.get("log"))
        if not build_elf.is_file() or build_elf.is_symlink():
            raise RuntimeError(f"Ackermann preset ELF is missing or linked: {expected_path}")
        build_bytes = build_elf.read_bytes()
        if row_bytes != len(build_bytes) or row_sha != hashlib.sha256(build_bytes).hexdigest():
            raise RuntimeError(f"Ackermann build summary hash/size does not match {preset} ELF.")
        if (not log_value or Path(log_value).is_absolute() or ".." in Path(log_value).parts):
            raise RuntimeError(f"Ackermann build log path is unsafe for {preset}.")
        log_path = source / ACKERMANN_DELIVERY_ROOT / log_value
        if not log_path.is_file() or log_path.is_symlink():
            raise RuntimeError(f"Ackermann build log is missing or linked for {preset}.")
        exitcode_value = normalize_relative_path(row.get("exitcode_file"))
        if (not exitcode_value or Path(exitcode_value).is_absolute() or
                ".." in Path(exitcode_value).parts):
            raise RuntimeError(f"Ackermann build exit-code path is unsafe for {preset}.")
        exitcode_path = source / ACKERMANN_DELIVERY_ROOT / exitcode_value
        if (not exitcode_path.is_file() or exitcode_path.is_symlink() or
                exitcode_path.read_text(encoding="utf-8-sig").strip() != "0"):
            raise RuntimeError(f"Ackermann build exit-code evidence is missing or failed for {preset}.")

    artifact_rows = artifact_manifest.get("artifacts")
    if isinstance(artifact_rows, dict):
        artifact_rows = list(artifact_rows.values())
    if not isinstance(artifact_rows, list) or len(artifact_rows) != 2:
        raise RuntimeError("Ackermann artifact manifest must contain exactly two ELF artifact rows.")
    artifacts_by_path = {}
    for row in artifact_rows:
        if not isinstance(row, dict):
            raise RuntimeError("Ackermann artifact manifest contains a malformed artifact row.")
        relative = normalize_relative_path(row.get("delivery_path", row.get("relative_path", row.get("path"))))
        if relative in artifacts_by_path:
            raise RuntimeError("Ackermann artifact manifest contains duplicate delivery paths.")
        artifacts_by_path[relative] = row

    for variant, spec in ACKERMANN_PROFILE_DELIVERIES.items():
        delivery_elf = source / spec["elf"]
        manifest_path = source / spec["manifest"]
        build_elf = source / "build" / spec["profile"] / "CarTest.elf"
        for path, label in ((delivery_elf, "delivery ELF"),
                            (manifest_path, "profile manifest"),
                            (build_elf, "preset ELF")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"Ackermann {variant} {label} is missing or linked: {path}")
        elf_bytes = delivery_elf.read_bytes()
        if elf_bytes != build_elf.read_bytes():
            raise RuntimeError(f"Ackermann {variant} delivery ELF differs from its named preset build.")
        elf_sha = hashlib.sha256(elf_bytes).hexdigest()
        try:
            profile_manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        except json.JSONDecodeError as error:
            raise RuntimeError(f"Ackermann {variant} profile manifest is invalid.") from error
        if (profile_manifest.get("schema_version") != 1 or
                profile_manifest.get("profile") not in (spec["profile"], spec["directory"]) or
                profile_manifest.get("preset", spec["profile"]) != spec["profile"] or
                profile_manifest.get("coordination_enabled") is not spec["coordination_enabled"] or
                profile_manifest.get("servo_slew_step_us") != spec["servo_slew_step_us"] or
                profile_manifest.get("elf") != "CarTest.elf" or
                profile_manifest.get("bytes") != len(elf_bytes) or
                profile_manifest.get("sha256") != elf_sha or
                normalize_relative_path(profile_manifest.get("build_log")) !=
                f"../build-logs/{spec['profile']}.log"):
            raise RuntimeError(f"Ackermann {variant} profile manifest does not match its ELF/contract.")
        profile_build_log = manifest_path.parent / profile_manifest["build_log"]
        ackermann_root = (source / ACKERMANN_DELIVERY_ROOT).resolve()
        resolved_build_log = profile_build_log.resolve()
        if (ackermann_root not in resolved_build_log.parents or
                not resolved_build_log.is_file() or profile_build_log.is_symlink()):
            raise RuntimeError(f"Ackermann {variant} profile build log is missing, linked, or escapes its package.")

    for required_delivery_doc in (
            ACKERMANN_PAIRED_DELIVERY_DOC, ACKERMANN_PAIRED_RUN_STEPS,
            ACKERMANN_TOOLING_EVIDENCE):
        path = source / required_delivery_doc
        if not path.is_file() or path.is_symlink():
            raise RuntimeError(f"Required Ackermann package documentation/evidence is missing or linked: {required_delivery_doc}")

        artifact_path = f"{spec['directory']}/CarTest.elf"
        artifact = artifacts_by_path.get(artifact_path)
        if (not artifact or artifact.get("preset") != spec["profile"] or
                artifact.get("coordination_enabled") is not spec["coordination_enabled"] or
                artifact.get("bytes") != len(elf_bytes) or artifact.get("sha256") != elf_sha):
            raise RuntimeError(f"Ackermann artifact manifest does not match {variant} ELF/coordination.")
        row = build_rows[spec["profile"]]
        row_path = normalize_relative_path(row.get("elf_path", row.get("elf")))
        row_bytes = row.get("elf_bytes", row.get("bytes"))
        row_sha = row.get("elf_sha256", row.get("sha256"))
        expected_build_path = f"build/{spec['profile']}/CarTest.elf"
        if row_path != expected_build_path or row_bytes != len(elf_bytes) or row_sha != elf_sha:
            raise RuntimeError(f"Ackermann build summary does not match {variant} ELF.")


def _require_regular_checkpoint_file(source: Path, relative_name: str, label: str) -> Path:
    path = source
    for part in Path(relative_name).parts:
        path = path / part
        if path.is_symlink():
            raise RuntimeError(f"OpenMV native-preview {label} is linked: {relative_name}")
    if not path.is_file():
        raise RuntimeError(f"OpenMV native-preview {label} is missing: {relative_name}")
    return path


def verify_openmv_native_preview_delivery(source: Path) -> list[str]:
    """Validate and return only the pinned, not-yet-flashed OPENMV4P firmware candidate."""
    candidate_root = source / OPENMV_NATIVE_PREVIEW_ROOT
    if candidate_root.is_symlink():
        raise RuntimeError("OpenMV native-preview delivery directory is linked.")

    required_paths = {
        relative_name: _require_regular_checkpoint_file(
            source, relative_name, "allowlisted artifact")
        for relative_name in OPENMV_NATIVE_PREVIEW_REQUIRED_FILES
    }
    manifest_path = required_paths[
        f"{OPENMV_NATIVE_PREVIEW_ROOT}/artifact-manifest.json"]
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("OpenMV native-preview artifact manifest is invalid.") from error
    if (not isinstance(manifest, dict) or
            manifest.get("run_id") != OPENMV_NATIVE_PREVIEW_RUN_ID or
            manifest.get("conclusion") != "success" or
            manifest.get("hardware_flashed") is not False or
            manifest.get("hardware_tested") is not False):
        raise RuntimeError("OpenMV native-preview run evidence is not the pinned successful, unflashed build.")

    manifest_hashes = manifest.get("files_sha256")
    if manifest_hashes != OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256:
        raise RuntimeError("OpenMV native-preview artifact manifest file/hash allowlist does not match the pinned build.")
    for name, expected_sha256 in OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256.items():
        relative_name = f"{OPENMV_NATIVE_PREVIEW_ROOT}/{name}"
        content = required_paths[relative_name].read_bytes()
        if hashlib.sha256(content).hexdigest() != expected_sha256:
            raise RuntimeError(f"OpenMV native-preview artifact hash mismatch: {name}")

    summary_path = required_paths[f"{OPENMV_NATIVE_PREVIEW_ROOT}/build-summary.txt"]
    try:
        summary_lines = summary_path.read_text(encoding="ascii").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise RuntimeError("OpenMV native-preview build summary is unreadable.") from error
    summary = {}
    for line in summary_lines:
        if not line or "=" not in line:
            raise RuntimeError("OpenMV native-preview build summary has an invalid row.")
        key, value = line.split("=", 1)
        if not key or key in summary:
            raise RuntimeError("OpenMV native-preview build summary has duplicate or empty keys.")
        summary[key] = value
    if summary != OPENMV_NATIVE_PREVIEW_EXPECTED_BUILD:
        raise RuntimeError("OpenMV native-preview tag, commit, SDK, patch, or image hashes do not match the pinned build.")

    summary_sha = hashlib.sha256(summary_path.read_bytes()).hexdigest()
    summary_sha_path = required_paths[
        f"{OPENMV_NATIVE_PREVIEW_ROOT}/build-summary.txt.sha256"]
    try:
        summary_sha_fields = summary_sha_path.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeDecodeError) as error:
        raise RuntimeError("OpenMV native-preview build-summary checksum is unreadable.") from error
    if summary_sha != OPENMV_NATIVE_PREVIEW_EXPECTED_FILES_SHA256["build-summary.txt"]:
        raise RuntimeError("OpenMV native-preview build-summary hash is not the pinned build.")
    if summary_sha_fields != [summary_sha, "build-summary.txt"]:
        raise RuntimeError("OpenMV native-preview build-summary checksum does not match the summary file.")

    patch_path = _require_regular_checkpoint_file(
        source, OPENMV_NATIVE_PREVIEW_PATCH, "pinned source patch")
    patch_sha = hashlib.sha256(patch_path.read_bytes()).hexdigest()
    if patch_sha != OPENMV_NATIVE_PREVIEW_EXPECTED_PATCH_SHA256:
        raise RuntimeError("OpenMV native-preview live pinned patch hash does not match the expected patch.")
    if summary.get("patch_sha256") != patch_sha:
        raise RuntimeError("OpenMV native-preview build summary patch hash does not match the live patch.")

    firmware_bin = required_paths[f"{OPENMV_NATIVE_PREVIEW_ROOT}/firmware.bin"].read_bytes()
    firmware_elf = required_paths[f"{OPENMV_NATIVE_PREVIEW_ROOT}/firmware.elf"].read_bytes()
    if len(firmware_bin) != OPENMV_NATIVE_PREVIEW_BIN_BYTES:
        raise RuntimeError("OpenMV native-preview firmware.bin size does not match the pinned image.")
    if len(firmware_elf) != OPENMV_NATIVE_PREVIEW_ELF_BYTES:
        raise RuntimeError("OpenMV native-preview firmware.elf size does not match the pinned image.")
    if (len(firmware_elf) < 52 or firmware_elf[:4] != b"\x7fELF" or
            firmware_elf[4] != 1 or firmware_elf[5] != 1 or firmware_elf[6] != 1):
        raise RuntimeError("OpenMV native-preview firmware.elf is not a little-endian ELF32 image.")
    elf_type, elf_machine, elf_version, elf_entry = struct.unpack_from("<HHII", firmware_elf, 16)
    if (elf_type != 2 or elf_machine != 40 or elf_version != 1 or
            elf_entry != OPENMV_NATIVE_PREVIEW_VECTOR_RESET or
            b"OPENMV4P" not in firmware_elf or b"STM32H743" not in firmware_elf):
        raise RuntimeError("OpenMV native-preview ELF target or entry point does not match OPENMV4P.")
    if len(firmware_bin) < 8:
        raise RuntimeError("OpenMV native-preview firmware.bin is too short for a vector table.")
    vector_sp, vector_reset = struct.unpack_from("<II", firmware_bin, 0)
    if (vector_sp != OPENMV_NATIVE_PREVIEW_VECTOR_SP or
            vector_reset != elf_entry or vector_reset != OPENMV_NATIVE_PREVIEW_VECTOR_RESET):
        raise RuntimeError("OpenMV native-preview firmware.bin vector table does not match the ELF.")

    allowlisted = list(OPENMV_NATIVE_PREVIEW_REQUIRED_FILES)
    for relative_name in OPENMV_NATIVE_PREVIEW_OPTIONAL_DOCUMENTS:
        path = source / relative_name
        if path.exists() or path.is_symlink():
            _require_regular_checkpoint_file(source, relative_name, "optional review document")
            allowlisted.append(relative_name)
    return allowlisted


def _require_uart_fifo_checkpoint_file(source: Path, relative_name: str, label: str) -> Path:
    path = source
    for part in Path(relative_name).parts:
        path = path / part
        if path.is_symlink():
            raise RuntimeError(f"OpenMV UART FIFO {label} is linked: {relative_name}")
    if not path.is_file():
        raise RuntimeError(f"OpenMV UART FIFO {label} is missing: {relative_name}")
    return path


def _parse_openmv_build_summary(path: Path) -> dict[str, str]:
    try:
        lines = path.read_text(encoding="ascii").splitlines()
    except (OSError, UnicodeDecodeError) as error:
        raise RuntimeError("OpenMV UART FIFO build summary is unreadable.") from error
    summary: dict[str, str] = {}
    for line in lines:
        if not line or "=" not in line:
            raise RuntimeError("OpenMV UART FIFO build summary has an invalid row.")
        key, value = line.split("=", 1)
        if not key or key in summary:
            raise RuntimeError("OpenMV UART FIFO build summary has duplicate or empty keys.")
        summary[key] = value
    return summary


def _matches_pinned_json(actual, expected: dict) -> bool:
    return json.dumps(actual, sort_keys=True, separators=(",", ":")) == json.dumps(
        expected, sort_keys=True, separators=(",", ":"))


def verify_openmv_uart_fifo_image_pair(firmware_bin: bytes, firmware_elf: bytes):
    """Verify the pinned OPENMV4P ELF entry and binary vector table agree."""
    if len(firmware_bin) < 8:
        raise RuntimeError("OpenMV UART FIFO firmware.bin is too short for a vector table.")
    if (len(firmware_elf) < 52 or firmware_elf[:4] != b"\x7fELF" or
            firmware_elf[4] != 1 or firmware_elf[5] != 1 or firmware_elf[6] != 1):
        raise RuntimeError("OpenMV UART FIFO firmware.elf is not a little-endian ELF32 image.")
    elf_type, elf_machine, elf_version, elf_entry = struct.unpack_from("<HHII", firmware_elf, 16)
    if (elf_type != 2 or elf_machine != 40 or elf_version != 1 or
            elf_entry != OPENMV_UART_FIFO_VECTOR_RESET or
            b"OPENMV4P" not in firmware_elf or b"STM32H743" not in firmware_elf):
        raise RuntimeError("OpenMV UART FIFO ELF target or entry point does not match OPENMV4P.")
    vector_sp, vector_reset = struct.unpack_from("<II", firmware_bin, 0)
    if (vector_sp != OPENMV_UART_FIFO_VECTOR_SP or
            vector_reset != elf_entry or vector_reset != OPENMV_UART_FIFO_VECTOR_RESET):
        raise RuntimeError("OpenMV UART FIFO firmware.bin vector table does not match the ELF.")


def verify_openmv_uart_fifo_firmware_delivery(source: Path) -> list[str]:
    """Validate and return only the pinned UART FIFO firmware and review files."""
    delivery_root = source / OPENMV_UART_FIFO_FIRMWARE_ROOT
    if delivery_root.is_symlink():
        raise RuntimeError("OpenMV UART FIFO delivery directory is linked.")

    required_paths = {
        relative_name: _require_uart_fifo_checkpoint_file(
            source, relative_name, "allowlisted file")
        for relative_name in OPENMV_UART_FIFO_FIRMWARE_FILES
    }
    manifest_path = required_paths[
        f"{OPENMV_UART_FIFO_FIRMWARE_ROOT}/artifact-manifest.json"]
    try:
        artifact_manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("OpenMV UART FIFO artifact manifest is invalid.") from error
    if (not isinstance(artifact_manifest, dict) or
            not _matches_pinned_json(
                artifact_manifest, OPENMV_UART_FIFO_EXPECTED_ARTIFACT_MANIFEST)):
        raise RuntimeError(
            "OpenMV UART FIFO artifact manifest metadata, CI commit, or hash pins do not match.")

    for name, expected in OPENMV_UART_FIFO_FIRMWARE_BUILD_FILES.items():
        relative_name = f"{OPENMV_UART_FIFO_FIRMWARE_ROOT}/{name}"
        content = required_paths[relative_name].read_bytes()
        if len(content) != expected["bytes"]:
            raise RuntimeError(f"OpenMV UART FIFO artifact size mismatch: {name}")
        if hashlib.sha256(content).hexdigest() != expected["sha256"]:
            raise RuntimeError(f"OpenMV UART FIFO artifact hash mismatch: {name}")

    summary_path = required_paths[
        f"{OPENMV_UART_FIFO_FIRMWARE_ROOT}/build-summary.txt"]
    if _parse_openmv_build_summary(summary_path) != OPENMV_UART_FIFO_FIRMWARE_BUILD_SUMMARY_EXPECTED:
        raise RuntimeError(
            "OpenMV UART FIFO build summary tag, commit, SDK, patch, API, or image hashes do not match.")
    summary_sha = hashlib.sha256(summary_path.read_bytes()).hexdigest()
    checksum_path = required_paths[
        f"{OPENMV_UART_FIFO_FIRMWARE_ROOT}/build-summary.txt.sha256"]
    try:
        checksum_fields = checksum_path.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeDecodeError) as error:
        raise RuntimeError("OpenMV UART FIFO build-summary checksum is unreadable.") from error
    if checksum_fields != [summary_sha, "build-summary.txt"]:
        raise RuntimeError("OpenMV UART FIFO build-summary checksum does not match the summary file.")

    patch_path = _require_uart_fifo_checkpoint_file(
        source, OPENMV_UART_FIFO_PATCH, "pinned source patch")
    patch_sha = hashlib.sha256(patch_path.read_bytes()).hexdigest()
    if patch_sha != OPENMV_UART_FIFO_PATCH_SHA256:
        raise RuntimeError("OpenMV UART FIFO live pinned patch hash does not match.")
    patch_manifest_path = _require_uart_fifo_checkpoint_file(
        source, OPENMV_UART_FIFO_PATCH_MANIFEST, "patch manifest")
    if hashlib.sha256(patch_manifest_path.read_bytes()).hexdigest() != OPENMV_UART_FIFO_PATCH_MANIFEST_SHA256:
        raise RuntimeError("OpenMV UART FIFO patch manifest hash does not match its pin.")
    try:
        patch_manifest = json.loads(patch_manifest_path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("OpenMV UART FIFO patch manifest is invalid.") from error
    if (not isinstance(patch_manifest, dict) or
            not _matches_pinned_json(
                patch_manifest, OPENMV_UART_FIFO_PATCH_MANIFEST_EXPECTED)):
        raise RuntimeError("OpenMV UART FIFO patch manifest does not match the pinned OpenMV/MP base.")
    patch_readme_path = _require_uart_fifo_checkpoint_file(
        source, OPENMV_UART_FIFO_PATCH_README, "patch README")
    if hashlib.sha256(patch_readme_path.read_bytes()).hexdigest() != OPENMV_UART_FIFO_PATCH_README_SHA256:
        raise RuntimeError("OpenMV UART FIFO patch README hash does not match its pin.")

    for name, expected_sha256 in OPENMV_UART_FIFO_FIRMWARE_REVIEW_DOCS_SHA256.items():
        relative_name = f"{OPENMV_UART_FIFO_FIRMWARE_ROOT}/{name}"
        if hashlib.sha256(required_paths[relative_name].read_bytes()).hexdigest() != expected_sha256:
            raise RuntimeError(f"OpenMV UART FIFO review document hash mismatch: {name}")

    firmware_bin = required_paths[
        f"{OPENMV_UART_FIFO_FIRMWARE_ROOT}/firmware.bin"].read_bytes()
    firmware_elf = required_paths[
        f"{OPENMV_UART_FIFO_FIRMWARE_ROOT}/firmware.elf"].read_bytes()
    verify_openmv_uart_fifo_image_pair(firmware_bin, firmware_elf)
    return list(OPENMV_UART_FIFO_FIRMWARE_FILES)


def select_openmv_uart_fifo_firmware(source: Path, include: bool) -> list[str]:
    """Keep the UART FIFO binary delivery opt-in; source/tooling remains separate."""
    if not include:
        return []
    return verify_openmv_uart_fifo_firmware_delivery(source)


def verify_openmv_ackermann_packages(source: Path) -> list[str]:
    """Return only the declared production runtime files for the paired camera packages."""
    allowlisted: list[str] = []
    shared_runtime_names: set[str] | None = None
    for profile in OPENMV_ACKERMANN_PACKAGES:
        relative_root = f"{OPENMV_ACKERMANN_ROOT}/{profile}"
        package_root = source / relative_root
        manifest_path = package_root / "PACKAGE_MANIFEST.json"
        readme_path = package_root / "README.md"
        for path, label in ((manifest_path, "manifest"), (readme_path, "README")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"OpenMV Ackermann {profile} {label} is missing or linked.")
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"OpenMV Ackermann {profile} manifest is invalid.") from error
        runtime_files = manifest.get("runtime_files")
        protocol = manifest.get("protocol")
        curve_trace = manifest.get("curve_trace")
        if (manifest.get("schema_version") != 1 or
                manifest.get("control_profile") != profile or
                manifest.get("config_profile_selector") != profile or
                not manifest.get("package_id") or not manifest.get("build_id") or
                manifest.get("runtime_file_count") != len(OPENMV_ACKERMANN_RUNTIME_FILES) or
                not isinstance(runtime_files, list) or
                len(runtime_files) != len(OPENMV_ACKERMANN_RUNTIME_FILES) or
                not isinstance(protocol, dict) or
                protocol.get("version") != 2 or
                protocol.get("frame_size_bytes") != 16 or
                protocol.get("curve_phase_offset") != 10 or
                protocol.get("reserved_zero_offset") != 11 or
                protocol.get("slowdown_request_permille_offset") != 12 or
                protocol.get("crc_offset") != 14 or
                not isinstance(curve_trace, dict) or
                curve_trace.get("field") != "curve_trace_v2_hex" or
                curve_trace.get("record_size_bytes") != 16 or
                curve_trace.get("camera_diag_record_size_bytes") != 32 or
                curve_trace.get("control_ext_v2_record_size_bytes") != 32):
            raise RuntimeError(f"OpenMV Ackermann {profile} manifest metadata is incomplete.")
        runtime_names: set[str] = set()
        for entry in runtime_files:
            name = entry.get("name") if isinstance(entry, dict) else None
            if (not isinstance(name, str) or Path(name).name != name or
                    not name.endswith(".py") or name in runtime_names or
                    name not in OPENMV_ACKERMANN_RUNTIME_FILES):
                raise RuntimeError(f"OpenMV Ackermann {profile} runtime list is unsafe, duplicated, or not allowlisted.")
            runtime_names.add(name)
            runtime_path = package_root / name
            if not runtime_path.is_file() or runtime_path.is_symlink():
                raise RuntimeError(f"OpenMV Ackermann {profile} runtime file is missing or linked: {name}")
            content = runtime_path.read_bytes()
            if (entry.get("size_bytes") != len(content) or
                    entry.get("sha256") != hashlib.sha256(content).hexdigest() or
                    entry.get("mpy_cross_exit_code") != 0):
                raise RuntimeError(f"OpenMV Ackermann {profile} runtime/compiler evidence mismatch: {name}")
            allowlisted.append(f"{relative_root}/{name}")
        if runtime_names != OPENMV_ACKERMANN_RUNTIME_FILES:
            raise RuntimeError(f"OpenMV Ackermann {profile} runtime set does not match the production allowlist.")
        if shared_runtime_names is None:
            shared_runtime_names = runtime_names
        elif runtime_names != shared_runtime_names:
            raise RuntimeError("OpenMV Ackermann profiles do not share the same runtime module set.")
        allowlisted.extend((f"{relative_root}/PACKAGE_MANIFEST.json", f"{relative_root}/README.md"))
    return allowlisted


def verify_openmv_speedpd_packages(source: Path) -> list[str]:
    """Validate and return only the explicit runtime files for the three camera variants."""
    allowlisted: list[str] = []
    shared_runtime_names: set[str] | None = None
    for profile in OPENMV_SPEEDPD_PACKAGES:
        relative_root = f"{OPENMV_SPEEDPD_ROOT}/{profile}"
        package_root = source / relative_root
        manifest_path = package_root / "PACKAGE_MANIFEST.json"
        readme_path = package_root / "README.md"
        for path, label in ((manifest_path, "manifest"), (readme_path, "README")):
            if not path.is_file() or path.is_symlink():
                raise RuntimeError(f"OpenMV SpeedPD {profile} {label} is missing or linked.")
        try:
            manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        except (OSError, json.JSONDecodeError) as error:
            raise RuntimeError(f"OpenMV SpeedPD {profile} manifest is invalid.") from error
        runtime_files = manifest.get("runtime_files")
        if (manifest.get("schema_version") != 1 or
                manifest.get("control_profile") != profile or
                not manifest.get("package_id") or not manifest.get("build_id") or
                manifest.get("runtime_file_count") != 20 or
                not isinstance(runtime_files, list) or len(runtime_files) != 20):
            raise RuntimeError(f"OpenMV SpeedPD {profile} manifest metadata is incomplete.")

        runtime_names: set[str] = set()
        for entry in runtime_files:
            name = entry.get("name") if isinstance(entry, dict) else None
            if (not isinstance(name, str) or Path(name).name != name or
                    not name.endswith(".py") or name in runtime_names):
                raise RuntimeError(f"OpenMV SpeedPD {profile} runtime file list is unsafe or duplicated.")
            runtime_names.add(name)
            runtime_path = package_root / name
            if not runtime_path.is_file() or runtime_path.is_symlink():
                raise RuntimeError(f"OpenMV SpeedPD {profile} runtime file is missing or linked: {name}")
            runtime_bytes = runtime_path.read_bytes()
            runtime_sha = hashlib.sha256(runtime_bytes).hexdigest()
            if (entry.get("size_bytes") != len(runtime_bytes) or
                    entry.get("sha256") != runtime_sha or
                    entry.get("mpy_cross_exit_code") != 0):
                raise RuntimeError(f"OpenMV SpeedPD {profile} runtime/compiler evidence mismatch: {name}")
            allowlisted.append(f"{relative_root}/{name}")

        if shared_runtime_names is None:
            shared_runtime_names = runtime_names
        elif runtime_names != shared_runtime_names:
            raise RuntimeError("OpenMV SpeedPD profiles do not share the same runtime module set.")
        allowlisted.extend((
            f"{relative_root}/PACKAGE_MANIFEST.json",
            f"{relative_root}/README.md",
        ))
    return allowlisted


def verify_openmv_usb60_candidate_delivery(source: Path) -> list[str]:
    """Validate the explicitly selected, not-yet-hardware-accepted USB60 package."""
    package_root = source
    for part in Path(OPENMV_USB60_CANDIDATE_ROOT).parts:
        package_root = package_root / part
        if package_root.is_symlink():
            raise RuntimeError("OpenMV USB60 candidate package path contains a symlink.")
    if not package_root.is_dir():
        raise RuntimeError("OpenMV USB60 candidate package directory is missing or linked.")

    expected_names = (set(OPENMV_USB60_CANDIDATE_RUNTIME_FILES) |
                      set(OPENMV_USB60_CANDIDATE_DOCUMENTS) |
                      {OPENMV_USB60_CANDIDATE_MANIFEST})
    actual_names: set[str] = set()
    for item in package_root.rglob("*"):
        relative = item.relative_to(package_root)
        if item.is_symlink():
            raise RuntimeError(f"OpenMV USB60 candidate contains a symlink: {relative}")
        if item.is_dir():
            raise RuntimeError(f"OpenMV USB60 candidate contains an unexpected directory: {relative}")
        if not item.is_file() or relative.parent != Path("."):
            raise RuntimeError(f"OpenMV USB60 candidate contains an unsafe path: {relative}")
        if relative.name in actual_names:
            raise RuntimeError(f"OpenMV USB60 candidate contains a duplicate filename: {relative.name}")
        actual_names.add(relative.name)
    if actual_names != expected_names:
        missing = sorted(expected_names - actual_names)
        extra = sorted(actual_names - expected_names)
        raise RuntimeError(
            f"OpenMV USB60 candidate file allowlist mismatch: missing={missing}, extra={extra}")

    manifest_path = package_root / OPENMV_USB60_CANDIDATE_MANIFEST
    try:
        manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("OpenMV USB60 candidate package manifest is invalid.") from error
    if (not isinstance(manifest, dict) or
            manifest.get("schema_version") != 1 or
            manifest.get("package_id") != OPENMV_USB60_CANDIDATE_PACKAGE_ID or
            manifest.get("runtime_file_count") != len(OPENMV_USB60_CANDIDATE_RUNTIME_FILES) or
            manifest.get("release_status") != OPENMV_USB60_CHECKPOINT_STATUS or
            manifest.get("production_openmv_unchanged") is not True or
            manifest.get("baseline_coordinated_manifest_sha256") !=
            OPENMV_USB60_BASELINE_COORDINATED_MANIFEST_SHA256):
        raise RuntimeError("OpenMV USB60 candidate package metadata is incomplete or claims hardware acceptance.")

    limits = manifest.get("limits")
    if (not isinstance(limits, list) or
            "No 60 FPS acceptance is claimed." not in limits):
        raise RuntimeError("OpenMV USB60 candidate must state that 60 FPS acceptance is pending.")

    baseline = manifest.get("baseline")
    baseline_h743_entries = baseline.get("h743_elfs") if isinstance(baseline, dict) else None
    if not isinstance(baseline_h743_entries, list) or len(baseline_h743_entries) != 2:
        raise RuntimeError("OpenMV USB60 candidate paired H743 baseline references are incomplete.")
    declared_h743_hashes: dict[str, str] = {}
    for entry in baseline_h743_entries:
        relative_name = normalize_relative_path(
            entry.get("path") if isinstance(entry, dict) else None)
        digest = entry.get("sha256") if isinstance(entry, dict) else None
        if (relative_name not in OPENMV_USB60_BASELINE_H743_ELFS or
                relative_name in declared_h743_hashes or
                digest != OPENMV_USB60_BASELINE_H743_ELFS[relative_name]):
            raise RuntimeError("OpenMV USB60 candidate H743 baseline reference is unsafe or changed.")
        declared_h743_hashes[relative_name] = digest
    if set(declared_h743_hashes) != set(OPENMV_USB60_BASELINE_H743_ELFS):
        raise RuntimeError("OpenMV USB60 candidate paired H743 baseline references are incomplete.")
    top_level_h743_entries = manifest.get("baseline_h743_elfs")
    if not isinstance(top_level_h743_entries, list) or len(top_level_h743_entries) != 2:
        raise RuntimeError("OpenMV USB60 candidate top-level H743 baseline references are incomplete.")
    top_level_h743_hashes: dict[str, str] = {}
    for entry in top_level_h743_entries:
        relative_name = normalize_relative_path(
            entry.get("path") if isinstance(entry, dict) else None)
        digest = entry.get("sha256") if isinstance(entry, dict) else None
        if (relative_name not in OPENMV_USB60_BASELINE_H743_ELFS or
                relative_name in top_level_h743_hashes or
                digest != OPENMV_USB60_BASELINE_H743_ELFS[relative_name]):
            raise RuntimeError("OpenMV USB60 candidate top-level H743 baseline reference is unsafe or changed.")
        top_level_h743_hashes[relative_name] = digest
    if top_level_h743_hashes != declared_h743_hashes:
        raise RuntimeError("OpenMV USB60 candidate H743 baseline references disagree.")

    hardware_validation = manifest.get("hardware_validation")
    if (not isinstance(hardware_validation, dict) or
            not OPENMV_USB60_HW_VALIDATION_FALSE_FIELDS.issubset(hardware_validation) or
            any(value is not False for value in hardware_validation.values())):
        raise RuntimeError("OpenMV USB60 candidate hardware-validation state must remain entirely pending.")

    runtime_files = manifest.get("runtime_files")
    if (not isinstance(runtime_files, list) or
            len(runtime_files) != len(OPENMV_USB60_CANDIDATE_RUNTIME_FILES)):
        raise RuntimeError("OpenMV USB60 candidate runtime manifest is incomplete.")
    runtime_hashes: dict[str, str] = {}
    for entry in runtime_files:
        name = entry.get("name") if isinstance(entry, dict) else None
        if (not isinstance(name, str) or "/" in name or "\\" in name or
                Path(name).name != name or name not in OPENMV_USB60_CANDIDATE_RUNTIME_FILES or
                name in runtime_hashes):
            raise RuntimeError("OpenMV USB60 candidate runtime list contains an unsafe or duplicate path.")
        runtime_path = package_root / name
        content = runtime_path.read_bytes()
        actual_sha = hashlib.sha256(content).hexdigest()
        if (entry.get("size_bytes") != len(content) or
                entry.get("sha256") != actual_sha):
            raise RuntimeError(f"OpenMV USB60 candidate runtime hash/size mismatch: {name}")
        runtime_hashes[name] = actual_sha
    if set(runtime_hashes) != OPENMV_USB60_CANDIDATE_RUNTIME_FILES:
        raise RuntimeError("OpenMV USB60 candidate runtime list does not match the fixed 19-module allowlist.")

    cross_compile = manifest.get("cross_compile")
    if (not isinstance(cross_compile, dict) or
            cross_compile.get("target") != "armv7emsp" or
            cross_compile.get("module_count") != len(OPENMV_USB60_CANDIDATE_RUNTIME_FILES) or
            cross_compile.get("all_exit_codes_zero") is not True or
            not isinstance(cross_compile.get("compiler_path"), str) or
            not cross_compile.get("compiler_path") or
            not isinstance(cross_compile.get("compiler_version"), str) or
            not cross_compile.get("compiler_version")):
        raise RuntimeError("OpenMV USB60 candidate mpy-cross metadata is incomplete or failed.")
    compiled_modules = cross_compile.get("modules")
    if (not isinstance(compiled_modules, list) or
            len(compiled_modules) != len(OPENMV_USB60_CANDIDATE_RUNTIME_FILES)):
        raise RuntimeError("OpenMV USB60 candidate mpy-cross module list is incomplete.")
    compiled_names: set[str] = set()
    for entry in compiled_modules:
        name = entry.get("name") if isinstance(entry, dict) else None
        compiled_sha = entry.get("compiled_sha256") if isinstance(entry, dict) else None
        if (not isinstance(name, str) or "/" in name or "\\" in name or
                Path(name).name != name or name not in runtime_hashes or
                name in compiled_names or type(entry.get("exit_code")) is not int or
                entry.get("exit_code") != 0 or
                entry.get("source_sha256") != runtime_hashes.get(name) or
                type(entry.get("compiled_size_bytes")) is not int or
                entry.get("compiled_size_bytes") <= 0 or
                not isinstance(compiled_sha, str) or
                not re.fullmatch(r"[0-9a-f]{64}", compiled_sha)):
            raise RuntimeError("OpenMV USB60 candidate mpy-cross evidence does not match its runtime source.")
        compiled_names.add(name)
    if compiled_names != OPENMV_USB60_CANDIDATE_RUNTIME_FILES:
        raise RuntimeError("OpenMV USB60 candidate mpy-cross list does not match the 19 runtime modules.")

    documents = manifest.get("documents")
    if not isinstance(documents, dict) or set(documents) != set(OPENMV_USB60_CANDIDATE_DOCUMENTS):
        raise RuntimeError("OpenMV USB60 candidate README/compile-summary hashes are missing.")
    for name in OPENMV_USB60_CANDIDATE_DOCUMENTS:
        document = package_root / name
        content = document.read_bytes()
        entry = documents[name]
        if (not isinstance(entry, dict) or entry.get("size_bytes") != len(content) or
                entry.get("sha256") != hashlib.sha256(content).hexdigest()):
            raise RuntimeError(f"OpenMV USB60 candidate document hash/size mismatch: {name}")
    try:
        compile_summary = json.loads(
            (package_root / "CROSS_COMPILE_SUMMARY.json").read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("OpenMV USB60 candidate cross-compile summary is invalid JSON.") from error
    if not isinstance(compile_summary, dict):
        raise RuntimeError("OpenMV USB60 candidate cross-compile summary must be an object.")
    summary_cross_compile = compile_summary.get("cross_compile")
    if (not isinstance(summary_cross_compile, dict) or
            summary_cross_compile.get("target") != cross_compile.get("target") or
            summary_cross_compile.get("module_count") != cross_compile.get("module_count") or
            summary_cross_compile.get("all_exit_codes_zero") is not True or
            summary_cross_compile.get("compiler_path") != cross_compile.get("compiler_path") or
            summary_cross_compile.get("compiler_version") != cross_compile.get("compiler_version")):
        raise RuntimeError("OpenMV USB60 candidate cross-compile summary does not match its manifest.")
    summary_modules = summary_cross_compile.get("modules")
    if not isinstance(summary_modules, list) or len(summary_modules) != len(compiled_modules):
        raise RuntimeError("OpenMV USB60 candidate cross-compile summary module rows are incomplete.")
    summary_modules_by_name = {}
    for entry in summary_modules:
        name = entry.get("name") if isinstance(entry, dict) else None
        if (not isinstance(name, str) or name not in OPENMV_USB60_CANDIDATE_RUNTIME_FILES or
                name in summary_modules_by_name):
            raise RuntimeError("OpenMV USB60 candidate cross-compile summary has unsafe/duplicate module rows.")
        summary_modules_by_name[name] = entry
    for entry in compiled_modules:
        summary_entry = summary_modules_by_name.get(entry["name"])
        if (summary_entry is None or
                summary_entry.get("source_sha256") != entry["source_sha256"] or
                summary_entry.get("compiled_sha256") != entry["compiled_sha256"] or
                summary_entry.get("compiled_size_bytes") != entry["compiled_size_bytes"] or
                type(summary_entry.get("exit_code")) is not int or
                summary_entry.get("exit_code") != entry["exit_code"]):
            raise RuntimeError("OpenMV USB60 candidate compile summary module differs from its manifest.")

    baseline_manifest_relative = (
        f"{OPENMV_ACKERMANN_ROOT}/coordinated/PACKAGE_MANIFEST.json")
    baseline_manifest_path = source
    for part in Path(baseline_manifest_relative).parts:
        baseline_manifest_path = baseline_manifest_path / part
        if baseline_manifest_path.is_symlink():
            raise RuntimeError("OpenMV USB60 candidate baseline coordinated manifest is linked.")
    if not baseline_manifest_path.is_file():
        raise RuntimeError("OpenMV USB60 candidate baseline coordinated manifest is missing or linked.")
    if hashlib.sha256(baseline_manifest_path.read_bytes()).hexdigest() != \
            OPENMV_USB60_BASELINE_COORDINATED_MANIFEST_SHA256:
        raise RuntimeError("OpenMV USB60 candidate baseline coordinated manifest hash changed.")
    try:
        baseline_manifest = json.loads(baseline_manifest_path.read_text(encoding="utf-8-sig"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError("OpenMV USB60 candidate baseline coordinated manifest is invalid.") from error
    if (not isinstance(baseline, dict) or
            baseline.get("package") != f"{OPENMV_ACKERMANN_ROOT}/coordinated" or
            baseline.get("package_manifest_sha256") !=
            OPENMV_USB60_BASELINE_COORDINATED_MANIFEST_SHA256):
        raise RuntimeError("OpenMV USB60 candidate baseline package reference changed.")
    expected_baseline_runtime = {
        entry.get("name"): (entry.get("size_bytes"), entry.get("sha256"))
        for entry in baseline_manifest.get("runtime_files", [])
        if isinstance(entry, dict)
    }
    declared_baseline_runtime = {
        entry.get("name"): (entry.get("size_bytes"), entry.get("sha256"))
        for entry in baseline.get("runtime_files", [])
        if isinstance(entry, dict)
    }
    if (set(expected_baseline_runtime) != OPENMV_USB60_CANDIDATE_RUNTIME_FILES or
            declared_baseline_runtime != expected_baseline_runtime):
        raise RuntimeError("OpenMV USB60 candidate baseline runtime manifest does not match the pinned package.")
    for relative_name, expected_sha in OPENMV_USB60_BASELINE_H743_ELFS.items():
        baseline_elf = source
        for part in Path(relative_name).parts:
            baseline_elf = baseline_elf / part
            if baseline_elf.is_symlink():
                raise RuntimeError(f"OpenMV USB60 paired H743 baseline ELF is linked: {relative_name}")
        if not baseline_elf.is_file():
            raise RuntimeError(f"OpenMV USB60 paired H743 baseline ELF is missing or linked: {relative_name}")
        if hashlib.sha256(baseline_elf.read_bytes()).hexdigest() != expected_sha:
            raise RuntimeError(f"OpenMV USB60 paired H743 baseline ELF hash changed: {relative_name}")

    return [
        *(f"{OPENMV_USB60_CANDIDATE_ROOT}/{name}"
          for name in sorted(OPENMV_USB60_CANDIDATE_RUNTIME_FILES)),
        *(f"{OPENMV_USB60_CANDIDATE_ROOT}/{name}"
          for name in OPENMV_USB60_CANDIDATE_DOCUMENTS),
        f"{OPENMV_USB60_CANDIDATE_ROOT}/{OPENMV_USB60_CANDIDATE_MANIFEST}",
    ]


def select_openmv_usb60_candidate(source: Path, status: str) -> list[str]:
    if status != OPENMV_USB60_CHECKPOINT_STATUS:
        raise ValueError(
            "The USB60 candidate may only be checkpointed with status "
            f"{OPENMV_USB60_CHECKPOINT_STATUS!r}.")
    return verify_openmv_usb60_candidate_delivery(source)


def verify_historical_ackermann_pair(source: Path):
    """Validate frozen restoration ELFs without equating them to new builds.

    The old validator intentionally tied the original release to its then-live
    build tree. A later release reuses those preset names for new source, so
    only the independently pinned historical payloads may identify this pair.
    """
    for relative_name, expected_sha in OPENMV_USB60_BASELINE_H743_ELFS.items():
        path = source / relative_name
        if (not path.is_file() or path.is_symlink() or
                hashlib.sha256(path.read_bytes()).hexdigest() != expected_sha):
            raise RuntimeError("Historical Ackermann restoration ELF is missing or changed: " + relative_name)
    for spec in ACKERMANN_PROFILE_DELIVERIES.values():
        elf = source / spec["elf"]
        manifest_path = source / spec["manifest"]
        if manifest_path.is_symlink():
            raise RuntimeError("Historical Ackermann manifest is linked")
        manifest = json.loads(manifest_path.read_text(encoding="utf-8-sig"))
        if (manifest.get("sha256") != hashlib.sha256(elf.read_bytes()).hexdigest() or
                manifest.get("bytes") != elf.stat().st_size or
                manifest.get("preset", spec["profile"]) != spec["profile"]):
            raise RuntimeError("Historical Ackermann manifest differs from its pinned ELF")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=PROJECT)
    parser.add_argument("--version", required=True)
    parser.add_argument("--branch", required=True)
    parser.add_argument("--status", required=True)
    parser.add_argument(
        "--include-openmv-usb60-candidate", action="store_true",
        help="Validate and include the explicit hardware-pending OpenMV USB60 candidate package.")
    parser.add_argument(
        "--include-openmv-uart-fifo-firmware", action="store_true",
        help="Validate and include the pinned OpenMV UART FIFO firmware delivery.")
    parser.add_argument("--publish", action="store_true")
    parser.add_argument("--hardware-actions", default="none",
                        choices=("none", "stationary-camera-upload-h743-flash-no-motor-start"),
                        help="Record completed hardware actions accurately in VERSION.json.")
    parser.add_argument("--track-profiles", type=Path,
                        help="Validate and include an explicit three-profile H750 delivery under deliverables.")
    args = parser.parse_args()
    for value in (args.version, args.branch):
        if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9._/-]*", value) or ".." in value or value.endswith("/"):
            raise ValueError("Use a plain version/branch name without traversal or Git special syntax.")
    source = args.source.resolve()
    if source == STORE.resolve() or STORE.resolve() in source.parents:
        raise ValueError("The source must be outside the checkpoint worktree.")
    if not (source / "Src" / "main.c").is_file() or not (source / "openmv" / "main.py").is_file():
        raise ValueError("A complete CarTest H743/OpenMV source tree is required.")
    presets = json.loads((source / "CMakePresets.json").read_text(encoding="utf-8-sig"))
    profile_names = sorted(p["name"] for p in presets.get("buildPresets", [])
                           if not p.get("hidden", False))
    for profile in profile_names:
        if not (source / "build" / profile / "CarTest.elf").is_file():
            raise RuntimeError(f"Build the declared profile before checkpointing: {profile}")
    verify_uphill_deliveries(source)
    verify_speedpd_deliveries(source)
    if args.track_profiles is None:
        verify_ackermann_deliveries(source, set(profile_names))
    else:
        verify_historical_ackermann_pair(source)
    openmv_speedpd_files = verify_openmv_speedpd_packages(source)
    openmv_ackermann_files = verify_openmv_ackermann_packages(source)
    openmv_native_preview_files = verify_openmv_native_preview_delivery(source)
    openmv_usb60_candidate_files = []
    if args.include_openmv_usb60_candidate:
        openmv_usb60_candidate_files = select_openmv_usb60_candidate(source, args.status)
    openmv_uart_fifo_firmware_files = select_openmv_uart_fifo_firmware(
        source, args.include_openmv_uart_fifo_firmware)
    track_profile_files = []
    if args.track_profiles is not None:
        try:
            from .build_track_profiles import public_delivery_files
        except ImportError:
            from build_track_profiles import public_delivery_files
        track_profile_files = public_delivery_files(source, args.track_profiles)
    STORE.mkdir(parents=True, exist_ok=True)
    if not (STORE / ".git").is_dir():
        run_git("init", "-b", "main")
    run_git("config", "core.autocrlf", "false")
    if run_git("status", "--porcelain"):
        raise RuntimeError("Checkpoint worktree has uncommitted changes; preserve them before retrying.")
    existing = run_git("for-each-ref", "--format=%(refname:short)", "refs/heads").splitlines()
    if args.branch in existing:
        run_git("switch", args.branch)
    elif existing:
        run_git("switch", "-c", args.branch)
    elif args.branch != "main":
        run_git("symbolic-ref", "HEAD", "refs/heads/" + args.branch)
    if run_git("tag", "--list", args.version):
        raise RuntimeError("This immutable version tag already exists; choose a new version.")
    target = STORE / "CarTest"
    safe_remove(target)
    target.mkdir()
    for directory in DIRS:
        if (source / directory).is_dir():
            copy_tree(
                source / directory,
                target / directory,
                excluded_subtrees=PUBLIC_COPY_EXCLUDED_SUBTREES.get(directory, ()),
            )
    for name in ROOT_FILES:
        if (source / name).is_file():
            shutil.copy2(source / name, target / name)
    for name in PUBLIC_TEST_FIXTURES:
        fixture = source / name
        if not fixture.is_file() or fixture.is_symlink():
            raise RuntimeError(f"Required public regression fixture is missing or linked: {name}")
        if name in PUBLIC_TEST_FIXTURE_SHA256 and \
                hashlib.sha256(fixture.read_bytes()).hexdigest() != PUBLIC_TEST_FIXTURE_SHA256[name]:
            raise RuntimeError(f"Pinned public regression fixture changed: {name}")
        destination = target / name
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(fixture, destination)
    for name in PUBLIC_DELIVERY_FILES:
        copy_allowlisted_file(source, target, name)
    for name in openmv_speedpd_files:
        copy_allowlisted_file(source, target, name)
    for name in openmv_ackermann_files:
        copy_allowlisted_file(source, target, name)
    for name in openmv_native_preview_files:
        copy_allowlisted_file(source, target, name)
    for name in openmv_usb60_candidate_files:
        copy_allowlisted_file(source, target, name)
    for name in openmv_uart_fifo_firmware_files:
        copy_allowlisted_file(source, target, name)
    for name in track_profile_files:
        copy_allowlisted_file(source, target, name)
    for profile in profile_names:
        elf = source / "build" / profile / "CarTest.elf"
        output = target / "build" / elf.parent.name / elf.name
        output.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(elf, output)
    manifest = {str(p.relative_to(target)).replace("\\", "/"):
                hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted(target.rglob("*")) if p.is_file()}
    (STORE / "SHA256SUMS.json").write_text(json.dumps(manifest, indent=2) + "\n", encoding="utf-8")
    (STORE / "VERSION.json").write_text(json.dumps({"version": args.version,
        "status": args.status, "branch": args.branch, "files": len(manifest),
        "hardware_actions": args.hardware_actions}, indent=2) + "\n", encoding="utf-8")
    (STORE / "README.md").write_text("# CarTest version checkpoints\n\n"
        "H743 source, OpenMV upload files, and paired ELF checkpoints.\n\n"
        "Select a tag to restore a complete version. Experimental versions are not vehicle acceptance.\n"
        "The live source directory is not modified by this checkpoint tool.\n", encoding="utf-8")
    (STORE / ".gitattributes").write_text("* -text\n", encoding="utf-8")
    user, token = authenticated_user() if args.publish else ({"login": "CarTest-checkpoint", "id": 0}, "")
    run_git("-c", "user.name=" + user["login"], "-c",
            f"user.email={user['id']}+{user['login']}@users.noreply.github.com",
            "add", "--all")
    run_git("-c", "user.name=" + user["login"], "-c",
            f"user.email={user['id']}+{user['login']}@users.noreply.github.com",
            "commit", "-m", f"Checkpoint {args.version}: {args.status}")
    run_git("tag", args.version)
    commit = run_git("rev-parse", "HEAD")
    if args.publish:
        repo = api(f"/repos/{user['login']}/{REPO_NAME}", token)
        if repo is None:
            repo = api("/user/repos", token, {"name": REPO_NAME, "private": False,
                "description": "STM32H743 CarTest and OpenMV version checkpoints", "auto_init": False})
        if repo.get("private") or repo.get("owner", {}).get("login") != user["login"]:
            raise RuntimeError("Repository visibility/ownership does not match the authorized public backup.")
        remote = repo["clone_url"]
        if "origin" in run_git("remote").splitlines():
            if run_git("remote", "get-url", "origin") != remote:
                raise RuntimeError("The existing origin differs from the authorized backup repository.")
        else:
            run_git("remote", "add", "origin", remote)
        run_git("push", "--atomic", "origin", args.branch, "refs/tags/" + args.version)
        remote_commit = run_git("ls-remote", "origin", "refs/tags/" + args.version).split()[0]
        if remote_commit != commit:
            raise RuntimeError("Remote version verification failed.")
        print(json.dumps({"repository": repo["html_url"], "version": args.version,
                          "commit": commit, "remote_verified": True, "files": len(manifest)}))
    else:
        print(json.dumps({"version": args.version, "commit": commit, "published": False}))


if __name__ == "__main__":
    main()
