# Protocol and export-tool validation

Validation date: 2026-10-23. These checks use host builds, synthetic protocol
frames/log records, and fake STM32CubeProgrammer CLIs. They do not contain or
represent a vehicle run, camera capture, board flash, or measured frame rate.

## Results

| Check | Result |
| --- | --- |
| `tests/host/run_tests.ps1 -COnly` | PASS — 70 cases, 7,316 assertions, 13 shared V2 vectors |
| `python tools/run_host_tests.py` | PASS — exit 0; includes ACKermann motor-output harness, 41 cases / 368 checks |
| `python tests/test_checkpoint_version.py` | PASS — 2 tests; exact H743 pair and 19-module OpenMV allowlist, profile mismatch and unsafe package cases rejected |
| Live H743 checkpoint verifier | PASS — all 12 preset ELFs/build logs/zero exit-code records, paired hashes, and 23 explicitly allowlisted delivery files verified |
| `python -m unittest tests.test_analyze_steering_chain` | PASS — 15 tests; V2 curve CRC, capture/sequence binding, partial trace and invalid observation covered |
| `python -m unittest openmv.tests.test_protocol_host` | PASS — 10 tests; preallocated packer, strict field validation and shared vectors |
| `python tests/test_run_status_reader.py` | PASS — 23 tests; ACK1 CRC/ranges, seqlock guards, generation wrap/zero, frozen-only one-row export |
| `python tests/test_runtime_log_export.py` | PASS — 3 version-contract tests: V1 descriptor/249-record run, V1 timing, V2 timing. One optional fixture suite skipped because its external ELF/buffer environment was not set. |
| `tests/test_read_run_status.ps1` against final coordinated ELF | PASS — ACK1 before/snapshot/after generation reads, CRC decode and retry/error paths |
| `tests/test_export_run_log.ps1` against final coordinated ELF | PASS — 27-read HotPlug sequence, separately exported one-row ACK1 snapshot, bad-CRC classification, IDLE/ACTIVE no-CSV cases |

The full host runner also passed the production ACKermann motor-output variant
(41 cases / 368 checks), RLOG snapshot harness, receive diagnostic, and trace
assertions. Its deliberately broken PRIMASK negative-control baseline failed
as expected; the production positive control passed.

## Artifact identity checked by wrappers

The wrapper tests used the final coordinated ELF from this package:

- 2,041,352 bytes; SHA-256
  `6370200c9b843cb1ecb5983b1eb886910a77eb145dff9f51cd119074bbd36c70`.
- Its ACK1 symbol/section is 64 bytes, 32-byte aligned. The test fixture is
  generated from ELF symbols and contains only synthetic state.
- The export test binds a synthetic RLOG V2 fixture to that ELF build ID. Its
  three records and generated ACK1 snapshot are test fixtures, not vehicle data.

The wrapper output and synthetic bytes remain in the local temporary test
directory and are intentionally not part of the package. Only this concise
validation record is included in the exact checkpoint allowlist.

The OpenMV allowlist test uses synthetic package files to verify that only the
19 named runtime modules and two package documents are returned, that both
packages share the same module set, and that V2/curve-trace metadata matches the
wire contract. The real OpenMV package manifests and file hashes must pass the
same verifier before checkpoint creation.
