# v25 OpenMV APP build environment repair — 2026-10-29

The restored offline environment now builds the camera native APP successfully.
Production v25 Python, vehicle H743 source/configuration/ELFs and PE7 remain
unchanged. No camera upload, board reset, firmware flash or motor operation was
performed. Hardware FPS, actual IDE display rate and road acceptance are pending.

## First actual error

The newest original artifact directory, `261029211118-17088-e6c559`, contained
only `runner-environment.txt`: Make never started, so there was no `build.log`
or recorded Make exit code. A new invocation with a complete terminal capture
reproduced the configuration's first error:

```text
mkdir: cannot create directory '//w': Read-only file system
```

The fresh build directory was mapped directly to W: and supplied as `/w`.
`canonical_future_path` split it into parent `/` and basename `w`, then joined
them with another slash, producing `//w`. MSYS interpreted that as a UNC host.
The helper now strips the parent trailing slash before joining, preserving
`/w`. Native CPython receives the correct absolute `W:\` path.

An older record (`20261029T103015Z-87-2870`) separately showed an archive command
failing with `Argument list too long`. Directly mapping the fresh directory to
W: retains the short paths required to avoid that previous problem. The fix
does not switch back to the long directory names or reuse old objects.

Original records were not deleted or edited. Scripts were archived before
repair, all preexisting artifact hashes were recorded, and source/configuration
plus paired ELF files were preserved before project-tool changes. No archive
was re-extracted.

## Fixes and validation

- Fixed mapped-root canonicalization in the historical clean APP runner.
- Preserved full pre-Make/Make/post-build stdout/stderr as `runner-console.log`,
  with a runner exit code even for failures before Make. Make outputs remain
  distinct from these wrapper records. Tee failures also propagate.
- Added `-RuntimeRoot` selection and restoration of its process environment.
  The restored runner now defaults to the staged v25 runtime when present,
  prints the selected path, and preserves the historical fallback otherwise.
- Staged a verified copy of v25's 23 sources, manifest and validation-only MPYs
  under the environment's project directory; no v16 Python replaced live v25.
- Saved relocatable runner sources under
  `tools/vendor-patches/openmv-v5.0.1/offline_runner/`. They require the existing
  restored environment; no compiler/SDK binaries are added to public source.
- Added drive-root, nested-future-path and invalid-target shell regression tests.
  Together with native delivery/checkpoint tests: 27 tests ran, 26 passed and
  one existing environment-dependent test skipped.
- Native delivery validation now distinguishes a software-only hardware-pending
  build from a historical bench bundle. Software-only manifests must explicitly
  reject hardware/flash/FPS acceptance and cannot borrow old device evidence.
  The historical evidence requirements remain unchanged for historical bundles.

## Two successful independent clean builds

| Run | Runtime reference | Make / runner exit | Objects / dependencies |
| --- | --- | --- | --- |
| `261029212240-38664-a72043` | historical v16, environment verification | 0 / 0 | 787 / 531 |
| `261029212656-54176-1cebfd` | exact v25 23-source manifest | 0 / 0 | 787 / 531 |

Both used separate newly created empty directories, five pinned patches,
OMV_ENABLE_BL=0, full QSTR generation and 15 required native API QSTRs.
The second run's runtime source mismatch count is zero. The 23 source and MPY
hashes agree with the already-published v25 candidate package.

The independent builds produced byte-identical BIN and ELF files:

- firmware.bin: 1,741,964 bytes;
  SHA256 `c693f1ce035f8ff12796e2b21c21d347d5ef8aac7b4a78f6343a089ad45dc154`.
- firmware.elf: 11,229,528 bytes;
  SHA256 `97da850cf07ee7101e2dd0563ce0a1f2daac3257ab26ced18096c24a07a305b4`.

The native APP does not freeze the supplied v16/v25 project main runtime as a
replacement for the external project sources. The runtime reference is checked
and recorded for pairing; the matching acceleration callers are unchanged.
Identical APP bytes across these two references do not imply their Python
packages or road behavior are identical.

## v25 candidate and remaining work

`deliverables/OpenMV_RingJoint_v25_NativeCandidate_20261029` contains the newly
built APP-only BIN/ELF, source/build/artifact/hash manifests and the pinned
historical j-image reference. It passed full delivery validation against
`deliverables/TrackProfiles_RingJoint_v25_20261029/OpenMV` (34 allowed files).
That Python directory remains the exact published v25 runtime.

The public build record aliases three equivalent raw-runner field names/labels
for the established delivery schema, documents these aliases and retains the
raw record hash. Original raw records and complete logs remain unmodified in
`D:/OMVEnv/runner/artifacts`. The historical rollback reference is not a backup
of the replacement camera. No historical flash/readback/FPS evidence is claimed
for this new package.

The application retains the historical OPENMV4P vendor layout compiled with an
STM32H743xx target for the user's H750 camera compatibility path. This label
does not change the physical H750 identity, and does not refer to the vehicle
H743 firmware. APP-only range: `0x08040000..0x08200000`, end exclusive; no
bootloader or ROMFS is included.

Under the user's current no-flash instruction, further work is limited to
software preparation. Remaining device steps are replacement-camera partition
verification and backup, separately authorized native APP deployment, native
ABI checks and complete v25 throughput tests with physical UART and actual IDE
fresh-image measurement, followed by ordinary turns/cross/left/right ring road
tests. The previous replacement-camera stock-firmware capacity measurements
(headless about 55FPS, SDK JPEG about 29FPS) are not results for this new APP.
The requirement remains greater than 60FPS, including actual IDE preview.
