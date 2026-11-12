# Relocatable offline APP runner

These source files belong in `runner/` under the restored environment root
(for example `D:/OMVEnv`). They require its existing `source/openmv`, `tools/`,
and `project/` dependencies. They do not extract archives, delete failed builds,
download dependencies, flash/reset a board or start motors.

The historical APP builder under `project/tools/vendor-patches/openmv-v5.0.1/`
must include the mapped-drive-root canonicalization fix: join a parent `/`
without producing `//w` (an MSYS UNC path). This repository includes that fix.

Run the restored environment from PowerShell:

```powershell
& D:/OMVEnv/runner/Build-OpenMV4P-AppOnly.ps1 -RuntimeRoot D:/OMVEnv/project/deliverables/TrackProfiles_RingJoint_v25_20261029/OpenMV
```

`-VerifyOnly` performs checks without invoking Make. Omitting `-RuntimeRoot`
selects the staged v25 runtime when present, otherwise the historical v16
reference. The selected path is printed explicitly. Supply
the complete 23-source runtime and its matching cross-compilation manifest.

Each build creates a unique empty directory, temporarily mapped directly to
W: to bound Windows archive command length, with the package mapped to P:.
Occupied drives are refused; alternative letters can be supplied. Temporary
drive mappings are removed only after verifying their targets. Existing build
directories and artifacts are preserved. Full pre-Make/Make/post-build output
is saved as `runner-console.log`, with the runner exit code, including failures
before Make. `build.log`/`build.exitcode` are produced only if Make starts.

The generated APP targets the historical vendor OPENMV4P layout. Matching a
source-level ABI or compiling successfully is not replacement-camera layout,
FPS, actual IDE preview or road acceptance. Hardware deployment is separate.
