param(
    [switch]$COnly
)

$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$logDirectory = Join-Path $PSScriptRoot "logs"
$vectorFile = Join-Path $projectRoot "tests\data\vision-protocol-v1.csv"
$reverseVectorFile = Join-Path $projectRoot "openmv\tests\data\reverse-control-v1.csv"
$sourceFiles = @(
    (Join-Path $projectRoot "App\commissioning.c"),
    (Join-Path $projectRoot "App\servo_safety.c"),
    (Join-Path $projectRoot "App\reverse_control.c"),
    (Join-Path $projectRoot "App\car_vision.c"),
    (Join-Path $projectRoot "App\encoder_diag.c"),
    (Join-Path $projectRoot "App\speed_observer.c"),
    (Join-Path $projectRoot "App\speed_pi.c"),
    (Join-Path $projectRoot "App\follow_speed_pi.c"),
    (Join-Path $projectRoot "App\runtime_log.c"),
    (Join-Path $projectRoot "App\run_block_reason.c"),
    (Join-Path $projectRoot "App\manual_run_button.c"),
    (Join-Path $projectRoot "App\follow_output.c"),
    (Join-Path $projectRoot "App\uart_rx_service.c"),
    (Join-Path $projectRoot "App\vision_protocol.cpp"),
    (Join-Path $PSScriptRoot "test_main.cpp"),
    $vectorFile,
    $reverseVectorFile
)

foreach ($path in $sourceFiles) {
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) {
        throw "Required host-test input is missing: $path"
    }
}

$vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) {
    throw "Visual Studio Installer vswhere.exe not found: $vswhere"
}

$vsInstallResult = & $vswhere -latest -products "*" `
    -requires "Microsoft.VisualStudio.Component.VC.Tools.x86.x64" `
    -property installationPath
$vsInstall = [string]($vsInstallResult | Select-Object -First 1)
$vsInstall = $vsInstall.Trim()
if ([string]::IsNullOrWhiteSpace($vsInstall)) {
    throw "No Visual Studio installation with the x64 C++ compiler was found."
}

$vcvars = Join-Path $vsInstall "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path -LiteralPath $vcvars -PathType Leaf)) {
    throw "Visual Studio x64 developer environment script not found: $vcvars"
}

New-Item -ItemType Directory -Force -Path $logDirectory | Out-Null
$runStamp = Get-Date -Format "yyyyMMdd-HHmmss-fff"
$logFile = Join-Path $logDirectory "msvc-host-$runStamp.log"
$runDirectory = Join-Path $env:TEMP "CarTestHostTests-$([guid]::NewGuid().ToString('N'))"
New-Item -ItemType Directory -Force -Path $runDirectory | Out-Null

$objectFiles = @(
    (Join-Path $runDirectory "commissioning.obj"),
    (Join-Path $runDirectory "servo_safety.obj"),
    (Join-Path $runDirectory "reverse_control.obj"),
    (Join-Path $runDirectory "car_vision.obj"),
    (Join-Path $runDirectory "encoder_diag.obj"),
    (Join-Path $runDirectory "runtime_log.obj"),
    (Join-Path $runDirectory "vision_protocol.obj"),
    (Join-Path $runDirectory "test_main.obj"),
    (Join-Path $runDirectory "uart_rx_service.obj"),
    (Join-Path $runDirectory "run_block_reason.obj"),
    (Join-Path $runDirectory "manual_run_button.obj"),
    (Join-Path $runDirectory "follow_output.obj"),
    (Join-Path $runDirectory "follow_output_turn.obj"),
    (Join-Path $runDirectory "speed_observer.obj"),
    (Join-Path $runDirectory "speed_pi.obj"),
    (Join-Path $runDirectory "follow_speed_pi.obj"),
    (Join-Path $runDirectory "servo_safety_turn.obj")
)
$testExecutable = Join-Path $runDirectory "CarTestHostTests.exe"
$includeArgument = "/I`"$(Join-Path $projectRoot 'Inc')`""
$runtimeLogBuildIdInclude = ""
$runtimeLogBuildIdDefine = ""
if (-not [string]::IsNullOrWhiteSpace($env:CARTEST_RUNTIME_LOG_FIXTURE_PATH)) {
    $runtimeLogBuildPreset = "DebugFollow"
    if (-not [string]::IsNullOrWhiteSpace($env:CARTEST_RUNTIME_LOG_BUILD_PRESET)) {
        $runtimeLogBuildPreset = $env:CARTEST_RUNTIME_LOG_BUILD_PRESET.Trim()
    }
    if ($runtimeLogBuildPreset -notin @("DebugFollow", "DebugFollowTurn")) {
        throw "CARTEST_RUNTIME_LOG_BUILD_PRESET must be DebugFollow or DebugFollowTurn."
    }
    $generatedDirectory = Join-Path $projectRoot "build\$runtimeLogBuildPreset\generated"
    $generatedBuildId = Join-Path $generatedDirectory "runtime_log_build_id.h"
    if (-not (Test-Path -LiteralPath $generatedBuildId -PathType Leaf)) {
        throw "Build $runtimeLogBuildPreset before requesting a target-matched host fixture: $generatedBuildId"
    }
    $runtimeLogBuildIdInclude = "/I`"$generatedDirectory`""
    $runtimeLogBuildIdDefine = "/DCAR_RUNTIME_LOG_HOST_TEST_USE_TARGET_BUILD_ID"
}

$batchLines = @(
    "@echo off",
    "call `"$vcvars`" >nul 2>&1",
    "if errorlevel 1 (echo ERROR: vcvars64.bat failed & exit /b 1)",
    "chcp 65001 >nul",
    "pushd `"$projectRoot`"",
    "if errorlevel 1 exit /b 1",
    "echo === Compile commissioning.c as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[0])`" `"$(Join-Path $projectRoot 'App\commissioning.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile isolated FollowTurn servo limiter as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /DCARTEST_FOLLOW_SERVO_SLEW_STEP_US=120 /Dcar_servo_safety_init=car_servo_turn_safety_init /Dcar_servo_safety_request=car_servo_turn_safety_request /Dcar_servo_safety_request_selected=car_servo_turn_safety_request_selected /Dcar_servo_safety_request_selected_with_slew=car_servo_turn_safety_request_selected_with_slew /Dcar_servo_safety_stop=car_servo_turn_safety_stop /Dcar_servo_safety_acknowledge=car_servo_turn_safety_acknowledge /Dcar_servo_safety_timer_clock_hz=car_servo_turn_safety_timer_clock_hz /Dcar_servo_safety_apply_fixed_hsi64_profile=car_servo_turn_safety_apply_fixed_hsi64_profile /Dcar_servo_safety_validate_timer=car_servo_turn_safety_validate_timer /Dcar_servo_safety_stop_action=car_servo_turn_safety_stop_action /Fo`"$($objectFiles[16])`" `"$(Join-Path $projectRoot 'App\servo_safety.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile servo_safety.c as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[1])`" `"$(Join-Path $projectRoot 'App\servo_safety.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile reverse_control.c as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[2])`" `"$(Join-Path $projectRoot 'App\reverse_control.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile car_vision.c as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[3])`" `"$(Join-Path $projectRoot 'App\car_vision.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile encoder_diag.c as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[4])`" `"$(Join-Path $projectRoot 'App\encoder_diag.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile runtime_log.c as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument $runtimeLogBuildIdInclude $runtimeLogBuildIdDefine /DCAR_RUNTIME_LOG_HOST_TEST /Fo`"$($objectFiles[5])`" `"$(Join-Path $projectRoot 'App\runtime_log.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile vision_protocol.cpp as C++17 ===",
    "cl /nologo /std:c++17 /EHsc /W4 /TP /c $includeArgument /Fo`"$($objectFiles[6])`" `"$(Join-Path $projectRoot 'App\vision_protocol.cpp')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile host assertions as C++17 ===",
    "cl /nologo /std:c++17 /EHsc /W4 /TP /c $includeArgument /Fo`"$($objectFiles[7])`" `"$(Join-Path $PSScriptRoot 'test_main.cpp')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile bounded UART FIFO service as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[8])`" `"$(Join-Path $projectRoot 'App\uart_rx_service.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile Follow differential and per-side ramp helper as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[11])`" `"$(Join-Path $projectRoot 'App\follow_output.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile isolated FollowTurn slope-assist helper as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /DCARTEST_FOLLOW_TURN_PROFILE_COMPILED=1 /DCARTEST_FOLLOW_NORMAL_LIMIT_PERMILLE=700 /DCARTEST_FOLLOW_NORMAL_RAMP_STEP_PERMILLE=40 /Dcar_follow_output_reset=car_follow_output_turn_reset /Dcar_follow_output_hold=car_follow_output_turn_hold /Dcar_follow_output_update=car_follow_output_turn_update /Dcar_follow_output_update_with_slope_assist=car_follow_output_turn_update_with_slope_assist /Dcar_follow_output_slope_assist_request=car_follow_output_turn_slope_assist_request /Fo`"$($objectFiles[12])`" `"$(Join-Path $projectRoot 'App\follow_output.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile read-only run block reason classifier as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[9])`" `"$(Join-Path $projectRoot 'App\run_block_reason.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile manual Follow toggle helper as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[10])`" `"$(Join-Path $projectRoot 'App\manual_run_button.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile encoder speed observer and disabled PI library as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[13])`" `"$(Join-Path $projectRoot 'App\speed_observer.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[14])`" `"$(Join-Path $projectRoot 'App\speed_pi.c')`"",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /DCARTEST_FOLLOW_SPEED_PI_ENABLED=1 /Fo`"$($objectFiles[15])`" `"$(Join-Path $projectRoot 'App\follow_speed_pi.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Link actual CarTest pure modules and host assertions ===",
    "cl /nologo /Fe:`"$testExecutable`" `"$($objectFiles[0])`" `"$($objectFiles[1])`" `"$($objectFiles[2])`" `"$($objectFiles[3])`" `"$($objectFiles[4])`" `"$($objectFiles[5])`" `"$($objectFiles[6])`" `"$($objectFiles[7])`" `"$($objectFiles[8])`" `"$($objectFiles[9])`" `"$($objectFiles[10])`" `"$($objectFiles[11])`" `"$($objectFiles[12])`" `"$($objectFiles[13])`" `"$($objectFiles[14])`" `"$($objectFiles[15])`" `"$($objectFiles[16])`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Run host assertions from CarTest project root ===",
    "`"$testExecutable`"",
    "set TEST_EXIT=%errorlevel%",
    "if `"%TEST_EXIT%`"==`"0`" if not `"%CARTEST_RUNTIME_LOG_FIXTURE_PATH%`"==`"`" (",
    "  echo === Emit target-build-ID matched RAM logger fixture ===",
    "  `"$testExecutable`" --emit-runtime-log-fixture `"%CARTEST_RUNTIME_LOG_FIXTURE_PATH%`"",
    "  if errorlevel 1 set TEST_EXIT=1",
    ")",
    "echo === Extract and compile production servo_apply PRIMASK negative and positive controls ===",
    "python tests\host\test_servo_apply_primask.py --negative-source test-data\runtime-log-ab-20261012\baseline\Src__main.c --source Src\main.c",
    "if errorlevel 1 set TEST_EXIT=1",
    "echo === Verify slope diagnostic decoding in runtime-log exporter ===",
    "python tests\host\test_runtime_log_export_flags.py",
    "if errorlevel 1 set TEST_EXIT=1",
    "echo === Test offline encoder-speed baseline analyzer with synthetic CSV ===",
    "python tests\host\test_speed_baseline_tool.py",
    "if errorlevel 1 set TEST_EXIT=1",
    "echo === Extract and compile production runtime-log snapshot IRQ harness ===",
    "python tests\host\test_runtime_log_snapshot.py --source Src\main.c",
    "if errorlevel 1 set TEST_EXIT=1",
    "echo === Verify production physical Follow-start reverse resync handoff ===",
    "python tests\host\test_manual_start_resync_main.py --source Src\main.c",
    "if errorlevel 1 set TEST_EXIT=1",
    "echo === Verify production Follow speed PI safety and ramp wiring ===",
    "python tests\host\test_follow_speed_pi_main.py --source Src\main.c",
    "if errorlevel 1 set TEST_EXIT=1",
    "echo === Execute production motor_apply speed PI zero-coast and guard paths ===",
    "python tests\host\test_follow_output_path.py --source Src\main.c --output-dir test-data\speed-pi-trial-20261014\after\host-motor-apply",
    "if errorlevel 1 set TEST_EXIT=1",
    "popd",
    "exit /b %TEST_EXIT%"
)
if ($COnly) {
    $pythonTestsStart = [Array]::IndexOf(
        $batchLines,
        "echo === Extract and compile production servo_apply PRIMASK negative and positive controls ===")
    if ($pythonTestsStart -lt 0) {
        throw "Could not locate Python-test boundary for -COnly host run."
    }
    $batchLines = @($batchLines[0..($pythonTestsStart - 1)] + @(
        "popd",
        "exit /b %TEST_EXIT%"
    ))
}
$batchFile = Join-Path $runDirectory "run-host-tests.cmd"
[System.IO.File]::WriteAllLines($batchFile, $batchLines, [System.Text.Encoding]::ASCII)
$capturedOutputFile = Join-Path $runDirectory "console-output.txt"
$outerBatchFile = Join-Path $runDirectory "capture-host-tests.cmd"
$outerBatchLines = @(
    "@echo off",
    "call `"$batchFile`" > `"$capturedOutputFile`" 2>&1",
    "set TEST_EXIT=%errorlevel%",
    "exit /b %TEST_EXIT%"
)
[System.IO.File]::WriteAllLines($outerBatchFile, $outerBatchLines, [System.Text.Encoding]::ASCII)

$header = @(
    "CarTest MSVC host test run",
    "Timestamp: $(Get-Date -Format o)",
    "Visual Studio: $vsInstall",
    "Developer environment: $vcvars",
    "Command: tools/run_host_tests.py -> tests/host/run_tests.ps1 -> vcvars64.bat + cl.exe",
    "Working directory: $projectRoot",
    "Shared protocol vector: $vectorFile",
    "Vector SHA-256: $((Get-FileHash -LiteralPath $vectorFile -Algorithm SHA256).Hash)",
    "Shared reverse-control vector: $reverseVectorFile",
    "Reverse-control vector SHA-256: $((Get-FileHash -LiteralPath $reverseVectorFile -Algorithm SHA256).Hash)",
    "App/ pure modules and runtime_log.c are compiled directly; the speed PI adapter tests run against the production module, and the servo_apply() body is extracted from baseline and production main.c and compiled against HAL mocks. No HAL, UART device, or STM32 board is executed.",
    ""
)
[System.IO.File]::WriteAllLines($logFile, $header, [System.Text.Encoding]::UTF8)
Write-Host "MSVC host test log: $logFile"

Push-Location $projectRoot
$savedErrorActionPreference = $ErrorActionPreference
$ErrorActionPreference = "Continue"
try {
    & $env:ComSpec /d /s /c $outerBatchFile
    $runExitCode = $LASTEXITCODE
}
finally {
    Pop-Location
    $ErrorActionPreference = $savedErrorActionPreference
}

$capturedOutput = [System.IO.File]::ReadAllText(
    $capturedOutputFile, [System.Text.Encoding]::UTF8)
if (-not [string]::IsNullOrEmpty($capturedOutput)) {
    [System.IO.File]::AppendAllText(
        $logFile, $capturedOutput, (New-Object System.Text.UTF8Encoding($false)))
    $capturedOutput -split "`r?`n" | ForEach-Object {
        if (-not [string]::IsNullOrEmpty($_)) { Write-Host $_ }
    }
}

Add-Content -LiteralPath $logFile -Value "HOST_RUNNER_EXIT=$runExitCode"
Write-Host "HOST_RUNNER_EXIT=$runExitCode"
exit $runExitCode
