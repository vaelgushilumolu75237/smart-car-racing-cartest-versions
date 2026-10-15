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
    (Join-Path $runDirectory "follow_output.obj")
)
$testExecutable = Join-Path $runDirectory "CarTestHostTests.exe"
$includeArgument = "/I`"$(Join-Path $projectRoot 'Inc')`""
$runtimeLogBuildIdInclude = ""
$runtimeLogBuildIdDefine = ""
if (-not [string]::IsNullOrWhiteSpace($env:CARTEST_RUNTIME_LOG_FIXTURE_PATH)) {
    $generatedDirectory = Join-Path $projectRoot "build\DebugFollow\generated"
    $generatedBuildId = Join-Path $generatedDirectory "runtime_log_build_id.h"
    if (-not (Test-Path -LiteralPath $generatedBuildId -PathType Leaf)) {
        throw "Build DebugFollow before requesting a target-matched host fixture: $generatedBuildId"
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
    "echo === Compile read-only run block reason classifier as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[9])`" `"$(Join-Path $projectRoot 'App\run_block_reason.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Compile manual Follow toggle helper as C11 ===",
    "cl /nologo /std:c11 /W4 /TC /c $includeArgument /Fo`"$($objectFiles[10])`" `"$(Join-Path $projectRoot 'App\manual_run_button.c')`"",
    "if errorlevel 1 exit /b %errorlevel%",
    "echo === Link actual CarTest pure modules and host assertions ===",
    "cl /nologo /Fe:`"$testExecutable`" `"$($objectFiles[0])`" `"$($objectFiles[1])`" `"$($objectFiles[2])`" `"$($objectFiles[3])`" `"$($objectFiles[4])`" `"$($objectFiles[5])`" `"$($objectFiles[6])`" `"$($objectFiles[7])`" `"$($objectFiles[8])`" `"$($objectFiles[9])`" `"$($objectFiles[10])`" `"$($objectFiles[11])`"",
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
    "popd",
    "exit /b %TEST_EXIT%"
)
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
    "App/ pure modules and runtime_log.c are compiled directly; the exact servo_apply() body is extracted from baseline and production main.c and compiled against HAL mocks. No HAL, UART device, or STM32 board is executed.",
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
