$ErrorActionPreference = "Stop"
Set-StrictMode -Version Latest

$projectRoot = (Resolve-Path (Join-Path $PSScriptRoot "..\..")).Path
$vswhere = "C:\Program Files (x86)\Microsoft Visual Studio\Installer\vswhere.exe"
if (-not (Test-Path -LiteralPath $vswhere -PathType Leaf)) {
    throw "Visual Studio Installer vswhere.exe not found: $vswhere"
}
$vsInstall = [string]((& $vswhere -latest -products "*" `
    -requires "Microsoft.VisualStudio.Component.VC.Tools.x86.x64" `
    -property installationPath | Select-Object -First 1))
$vsInstall = $vsInstall.Trim()
if ([string]::IsNullOrWhiteSpace($vsInstall)) {
    throw "No Visual Studio installation with the x64 C++ compiler was found."
}
$vcvars = Join-Path $vsInstall "VC\Auxiliary\Build\vcvars64.bat"
if (-not (Test-Path -LiteralPath $vcvars -PathType Leaf)) {
    throw "Visual Studio x64 developer environment script not found: $vcvars"
}

$runDirectory = Join-Path $env:TEMP "CarTestAckermannTests-$([guid]::NewGuid().ToString('N'))"
New-Item -ItemType Directory -Path $runDirectory | Out-Null
$executable = Join-Path $runDirectory "CarTestAckermannTests.exe"
$batchFile = Join-Path $runDirectory "run-tests.cmd"
$logDirectory = Join-Path $PSScriptRoot "logs"
New-Item -ItemType Directory -Force -Path $logDirectory | Out-Null
$logFile = Join-Path $logDirectory ("ackermann-control-{0}.log" -f (Get-Date -Format "yyyyMMdd-HHmmss-fff"))

$batchLines = @(
    "@echo off",
    "call `"$vcvars`" >nul 2>&1",
    "if errorlevel 1 (echo ERROR: vcvars64.bat failed & exit /b 1)",
    "chcp 65001 >nul",
    "pushd `"$projectRoot`"",
    "if errorlevel 1 exit /b 1",
    "cl /nologo /std:c11 /W4 /WX /TC /DCAR_ACKERMANN_SNAPSHOT_HOST_TEST /DCARTEST_FOLLOW_INCREMENTAL_PI_ENABLED=1 /I`"Inc`" /Fo`"$runDirectory/`" tests\host\test_ackermann_control.c App\ackermann_control.c App\ackermann_snapshot.c App\follow_incremental_pi.c /Fe:`"$executable`"",
    "if errorlevel 1 (popd & exit /b 1)",
    "`"$executable`"",
    "if errorlevel 1 (popd & exit /b 1)"
)

$trackProfiles = @(
    @{ Name = "legacy"; Id = 0; Fastest = 4800; Slowest = 4000; ServoStep = 160 },
    @{ Name = "DebugFastNoRing"; Id = 1; Fastest = 5600; Slowest = 4400; ServoStep = 160 },
    @{ Name = "DebugRingBase"; Id = 2; Fastest = 4800; Slowest = 4000; ServoStep = 160 },
    @{ Name = "DebugRingFast"; Id = 3; Fastest = 5600; Slowest = 4400; ServoStep = 160 }
)
foreach ($profile in $trackProfiles) {
    $profileExe = Join-Path $runDirectory ("track-profile-{0}.exe" -f $profile.Name)
    $batchLines += @(
        "echo === Track target/snapshot/PI matrix: $($profile.Name) ===",
        "cl /nologo /std:c11 /W4 /WX /TC /DCAR_ACKERMANN_SNAPSHOT_HOST_TEST /DCARTEST_FOLLOW_INCREMENTAL_PI_ENABLED=1 /DCARTEST_FOLLOW_SERVO_SLEW_STEP_US=$($profile.ServoStep) /DCARTEST_TRACK_PROFILE=$($profile.Id) /DCARTEST_ACKERMANN_FASTEST_STRAIGHT_MILLIRPS=$($profile.Fastest) /DCARTEST_ACKERMANN_SLOWEST_REQUEST_MILLIRPS=$($profile.Slowest) /I`"Inc`" /Fo`"$runDirectory/`" tests\host\test_track_profile_targets.c App\ackermann_control.c App\ackermann_snapshot.c App\follow_incremental_pi.c /Fe:`"$profileExe`"",
        "if errorlevel 1 (popd & exit /b 1)",
        "`"$profileExe`"",
        "if errorlevel 1 (popd & exit /b 1)"
    )
}
$batchLines += @(
    "set TEST_EXIT=%errorlevel%",
    "popd",
    "exit /b %TEST_EXIT%"
)
Set-Content -LiteralPath $batchFile -Value $batchLines -Encoding ASCII

$output = @(& $env:ComSpec /d /c "`"$batchFile`"" 2>&1)
$exitCode = $LASTEXITCODE
$output | Set-Content -LiteralPath $logFile -Encoding UTF8
$output | ForEach-Object { Write-Output $_ }
Write-Output "Log: $logFile"
if ($exitCode -ne 0) {
    throw "Ackermann control host tests failed with exit code $exitCode."
}
