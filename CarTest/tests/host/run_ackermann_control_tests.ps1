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
