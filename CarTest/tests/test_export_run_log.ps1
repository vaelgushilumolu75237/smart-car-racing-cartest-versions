[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Elf,
    [Parameter(Mandatory = $true)][string]$Fixture,
    [Parameter(Mandatory = $true)][string]$OutputRoot
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$projectRoot = Split-Path -Parent $PSScriptRoot
$exportScript = Join-Path $projectRoot 'tools\export_run_log.ps1'
$validator = Join-Path $projectRoot 'tools\runtime_log_export.py'
$fakeSource = Join-Path $PSScriptRoot 'support\FakeCubeProgrammer.cs'
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
New-Item -ItemType Directory -Path $OutputRoot -Force | Out-Null
$fakeDirectory = Join-Path $OutputRoot 'fake cli with spaces'
New-Item -ItemType Directory -Path $fakeDirectory -Force | Out-Null
$fakeExe = Join-Path $fakeDirectory 'STM32_Programmer_CLI.exe'
Add-Type -TypeDefinition (Get-Content -LiteralPath $fakeSource -Raw -Encoding UTF8) `
    -OutputAssembly $fakeExe -OutputType ConsoleApplication

$pythonCommand = Get-Command -Name 'python.exe' -ErrorAction Stop
$inspectOutput = @(& $pythonCommand.Source $validator inspect --elf (Resolve-Path -LiteralPath $Elf).Path)
if ($LASTEXITCODE -ne 0) {
    throw 'Python could not inspect the selected ELF for the fake-CLI integration test.'
}
$layout = ($inspectOutput -join [Environment]::NewLine) | ConvertFrom-Json
$serial = '1234567890'
$powershellExe = Join-Path $PSHOME 'powershell.exe'

function Invoke-FakeExport {
    param([string]$Name, [string]$Mode, [string]$Frequency = '4000', [bool]$Shared = $false)
    $caseRoot = Join-Path $OutputRoot $Name
    New-Item -ItemType Directory -Path $caseRoot -Force | Out-Null
    $argsLog = Join-Path $caseRoot 'fake-cli-args.tsv'
    $callIndex = Join-Path $caseRoot 'fake-cli-call-index.txt'
    $env:RUNTIME_LOG_FAKE_BUFFER_PATH = (Resolve-Path -LiteralPath $Fixture).Path
    $env:RUNTIME_LOG_FAKE_ARGS_PATH = $argsLog
    $env:RUNTIME_LOG_FAKE_CALL_INDEX_PATH = $callIndex
    $env:RUNTIME_LOG_FAKE_EXPECTED_ADDRESS = [string]$layout.storage_address
    $env:RUNTIME_LOG_FAKE_EXPECTED_SN = $serial
    $env:RUNTIME_LOG_FAKE_FREQ = $Frequency
    $env:RUNTIME_LOG_FAKE_SHARED = if ($Shared) { '1' } else { '0' }
    $env:RUNTIME_LOG_FAKE_MODE = $Mode

    $childArguments = @(
        '-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
        '-File', $exportScript,
        '-Elf', (Resolve-Path -LiteralPath $Elf).Path,
        '-OutDirectory', (Join-Path $caseRoot 'output root with spaces'),
        '-ProgrammerCLI', $fakeExe,
        '-ProbeSN', $serial,
        '-ConnectionFrequencyKhz', $Frequency
    )
    if ($Shared) { $childArguments += '-SharedConnection' }
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $childOutput = @(& $powershellExe @childArguments 2>&1)
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }
    $text = (($childOutput | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine)
    Set-Content -LiteralPath (Join-Path $caseRoot 'wrapper-output.log') -Value $text -Encoding UTF8
    return [PSCustomObject]@{ CaseRoot = $caseRoot; ArgsLog = $argsLog; ExitCode = $exitCode; Output = $text }
}

function Get-ExportRunDirectory {
    param([string]$CaseRoot)
    $root = Join-Path $CaseRoot 'output root with spaces'
    return Get-ChildItem -LiteralPath $root -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1
}

function Assert-HotPlugUploadArguments {
    param([string]$ArgsLog, [string]$Frequency = '4000', [bool]$Shared = $false)
    $lines = @(Get-Content -LiteralPath $ArgsLog -Encoding UTF8)
    if ($lines.Count -ne 3) {
        throw "Expected exactly three fake CLI uploads, found $($lines.Count)."
    }
    $expectedSizes = @(64, [int](64 + ([int]$fixtureHeader.count * 64)), 64)
    $expectedUploadIndex = 4 + [int]$Shared + 1
    $expectedCount = $expectedUploadIndex + 4
    for ($index = 0; $index -lt $lines.Count; ++$index) {
        $tokens = $lines[$index].Split([char]9)
        if ($tokens.Count -ne $expectedCount -or $tokens[0] -ne '-c' -or
            $tokens[1] -ne 'port=SWD' -or $tokens[2] -ne 'mode=HOTPLUG' -or
            $tokens[3] -ne "freq=$Frequency" -or
            ($Shared -and $tokens[4] -ne 'shared') -or
            $tokens[$expectedUploadIndex - 1] -ne "sn=$serial" -or
            $tokens[$expectedUploadIndex] -ne '--upload' -or
            $tokens[$expectedUploadIndex + 1] -ne $layout.storage_address -or
            [int]$tokens[$expectedUploadIndex + 2] -ne $expectedSizes[$index]) {
            throw "Fake CLI call $($index + 1) did not match the HOTPLUG upload allowlist: $($lines[$index])"
        }
        $forbidden = @('port=JTAG', 'mode=NORMAL', '-rst', '-halt', '--download', '--erase', '-go', '--run')
        foreach ($token in $tokens) {
            if ($forbidden -contains $token) {
                throw "Fake CLI observed forbidden command argument '$token'."
            }
        }
    }
}

$fixtureHeaderPath = Join-Path $OutputRoot 'fixture-header.bin'
$fixtureBytes = [IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $Fixture).Path)
$fixtureHeader = New-Object byte[] 64
[Array]::Copy($fixtureBytes, 0, $fixtureHeader, 0, 64)
[IO.File]::WriteAllBytes($fixtureHeaderPath, $fixtureHeader)
$fixtureHeaderJson = @(& $pythonCommand.Source $validator check-header --elf (Resolve-Path -LiteralPath $Elf).Path --header $fixtureHeaderPath)
if ($LASTEXITCODE -ne 0) {
    throw 'Python could not validate the real logger fixture header.'
}
$fixtureHeader = ($fixtureHeaderJson -join [Environment]::NewLine) | ConvertFrom-Json

try {
    $success = Invoke-FakeExport -Name 'warning-stderr-success' -Mode 'warning-stderr'
    if ($success.ExitCode -ne 0) {
        throw "Fake warning-stderr run should succeed; got exit $($success.ExitCode). $($success.Output)"
    }
    Assert-HotPlugUploadArguments -ArgsLog $success.ArgsLog
    $successRun = Get-ExportRunDirectory -CaseRoot $success.CaseRoot
    $successCsv = Join-Path $successRun.FullName 'run.csv'
    $successCliLog = Join-Path $successRun.FullName 'cubeprogrammer-payload.log'
    $successMetadataPath = Join-Path $successRun.FullName 'run-metadata.json'
    if (-not (Test-Path -LiteralPath $successCsv -PathType Leaf) -or
        -not (Test-Path -LiteralPath $successCliLog -PathType Leaf) -or
        -not (Test-Path -LiteralPath $successMetadataPath -PathType Leaf)) {
        throw 'Successful fake export did not create its CSV, CLI log, and metadata.'
    }
    $successMetadata = Get-Content -LiteralPath $successMetadataPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $metadataProperties = @($successMetadata.PSObject.Properties.Name)
    if ($successMetadata.status -ne 'success' -or
        $successMetadata.record_count -ne $fixtureHeader.count -or
        $successMetadata.csv -ne $successCsv -or
        $successMetadata.planned_csv_path -ne $successCsv -or
        -not ($metadataProperties -contains 'buffer_full') -or
        -not ($metadataProperties -contains 'records_truncated')) {
        throw 'Successful metadata is missing the verified record/full/truncation fields.'
    }
    if ($successMetadata.connection_frequency_khz -ne 4000 -or
        $successMetadata.shared_connection -ne $false -or
        $successMetadata.connection -ne 'SWD HOTPLUG freq=4000') {
        throw 'Default export metadata did not retain the exclusive 4000 kHz connection settings.'
    }
    $successLogText = Get-Content -LiteralPath $successCliLog -Raw -Encoding UTF8
    if ($successLogText -notmatch 'warning written to stderr' -or
        $successLogText -notmatch 'fake CLI uploaded') {
        throw 'Successful CLI log did not preserve both stderr warning and stdout output.'
    }

    $failed = Invoke-FakeExport -Name 'stderr-nonzero-failure' -Mode 'fail-stderr'
    if ($failed.ExitCode -eq 0) {
        throw 'Fake CLI nonzero stderr failure unexpectedly succeeded.'
    }
    $failedRun = Get-ExportRunDirectory -CaseRoot $failed.CaseRoot
    $failedMetadataPath = Join-Path $failedRun.FullName 'run-metadata.json'
    $failedCliLog = Join-Path $failedRun.FullName 'cubeprogrammer-header-initial.log'
    if (-not (Test-Path -LiteralPath $failedMetadataPath -PathType Leaf) -or
        -not (Test-Path -LiteralPath $failedCliLog -PathType Leaf) -or
        (Test-Path -LiteralPath (Join-Path $failedRun.FullName 'run.csv'))) {
        throw 'Failed CLI run did not retain failure metadata/log or created a success CSV.'
    }
    $failedLogText = Get-Content -LiteralPath $failedCliLog -Raw -Encoding UTF8
    if ($failedLogText -notmatch 'failure written to stderr') {
        throw 'Nonzero CLI stderr output was not retained in its log.'
    }

    $mutated = Invoke-FakeExport -Name 'mutated-final-header-failure' -Mode 'mutate-final-header'
    if ($mutated.ExitCode -eq 0) {
        throw 'Fake final-header mutation unexpectedly succeeded.'
    }
    $mutatedRun = Get-ExportRunDirectory -CaseRoot $mutated.CaseRoot
    if ((Test-Path -LiteralPath (Join-Path $mutatedRun.FullName 'run.csv')) -or
        -not (Test-Path -LiteralPath (Join-Path $mutatedRun.FullName 'runtime-log.bin')) -or
        -not (Test-Path -LiteralPath (Join-Path $mutatedRun.FullName 'run-metadata.json'))) {
        throw 'Header mutation failure did not retain raw bytes/metadata or created CSV.'
    }

    $shared = Invoke-FakeExport -Name 'shared-low-frequency-success' -Mode 'warning-stderr' -Frequency '1000' -Shared $true
    if ($shared.ExitCode -ne 0) {
        throw "Fake shared 1000 kHz run should succeed; got exit $($shared.ExitCode). $($shared.Output)"
    }
    Assert-HotPlugUploadArguments -ArgsLog $shared.ArgsLog -Frequency '1000' -Shared $true
    $sharedRun = Get-ExportRunDirectory -CaseRoot $shared.CaseRoot
    $sharedMetadata = Get-Content -LiteralPath (Join-Path $sharedRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($sharedMetadata.connection_frequency_khz -ne 1000 -or
        $sharedMetadata.shared_connection -ne $true -or
        $sharedMetadata.connection -ne 'SWD HOTPLUG freq=1000 shared') {
        throw 'Shared 1000 kHz settings were not retained in export metadata.'
    }

    $connectError = Invoke-FakeExport -Name 'dev-connect-error-no-fallback' -Mode 'connect-error' -Frequency '1000' -Shared $true
    if ($connectError.ExitCode -eq 0) {
        throw 'Fake DEV_CONNECT_ERR unexpectedly succeeded.'
    }
    $connectRun = Get-ExportRunDirectory -CaseRoot $connectError.CaseRoot
    $connectMetadata = Get-Content -LiteralPath (Join-Path $connectRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $connectArgs = @(Get-Content -LiteralPath $connectError.ArgsLog -Encoding UTF8)
    if ($connectArgs.Count -ne 1 -or
        $connectMetadata.status -ne 'failed' -or
        $null -ne $connectMetadata.csv -or
        [string]::IsNullOrWhiteSpace([string]$connectMetadata.planned_csv_path) -or
        $null -ne $connectMetadata.initial_header -or
        $null -ne $connectMetadata.raw_sha256 -or
        $connectMetadata.error -notmatch 'DEV_CONNECT_ERR' -or
        $connectMetadata.error -notmatch 'does not stop the vehicle' -or
        (Test-Path -LiteralPath (Join-Path $connectRun.FullName 'run.csv'))) {
        throw 'DEV_CONNECT_ERR must preserve no-data metadata and fail closed without automatic retry/fallback.'
    }

    $mutatedMetadata = Get-Content -LiteralPath (Join-Path $mutatedRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($null -ne $mutatedMetadata.csv -or
        [string]::IsNullOrWhiteSpace([string]$mutatedMetadata.planned_csv_path)) {
        throw 'Header-change failure metadata must distinguish the absent CSV from its planned path.'
    }

    $decodeFailure = Invoke-FakeExport -Name 'decode-error-no-csv' -Mode 'decode-error'
    if ($decodeFailure.ExitCode -eq 0) {
        throw 'Corrupted synthetic payload unexpectedly decoded as a successful CSV.'
    }
    $decodeFailureRun = Get-ExportRunDirectory -CaseRoot $decodeFailure.CaseRoot
    $decodeFailureMetadata = Get-Content -LiteralPath (Join-Path $decodeFailureRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($decodeFailureMetadata.status -ne 'failed' -or
        $null -ne $decodeFailureMetadata.csv -or
        [string]::IsNullOrWhiteSpace([string]$decodeFailureMetadata.planned_csv_path) -or
        (Test-Path -LiteralPath (Join-Path $decodeFailureRun.FullName 'run.csv'))) {
        throw 'Decode failure metadata must leave csv null and retain the planned path separately.'
    }

    $dryRunRoot = Join-Path $OutputRoot 'dry run shared frequency'
    $dryRunArguments = @(
        '-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
        '-File', $exportScript,
        '-Elf', (Resolve-Path -LiteralPath $Elf).Path,
        '-OutDirectory', $dryRunRoot,
        '-ProgrammerCLI', $fakeExe,
        '-ProbeSN', $serial,
        '-ConnectionFrequencyKhz', '1000',
        '-SharedConnection',
        '-DryRun'
    )
    $dryRunOutput = @(& $powershellExe @dryRunArguments)
    if ($LASTEXITCODE -ne 0) { throw 'Dry-run connection parameter case failed.' }
    $dryPlan = ($dryRunOutput -join [Environment]::NewLine) | ConvertFrom-Json
    $expectedConnectionArguments = @('-c', 'port=SWD', 'mode=HOTPLUG', 'freq=1000', 'shared', "sn=$serial")
    foreach ($readName in @('first_read', 'second_read', 'final_read')) {
        $readArguments = @($dryPlan.$readName)
        if ($readArguments.Count -lt ($expectedConnectionArguments.Count + 4)) {
            throw "Dry-run $readName is missing its upload arguments."
        }
        for ($index = 0; $index -lt $expectedConnectionArguments.Count; ++$index) {
            if ($readArguments[$index] -ne $expectedConnectionArguments[$index]) {
                throw "Dry-run $readName did not show the selected shared 1000 kHz connection."
            }
        }
        if ($readArguments[$expectedConnectionArguments.Count] -ne '--upload') {
            throw "Dry-run $readName is not restricted to RAM upload."
        }
    }
    if ($dryPlan.connection_frequency_khz -ne 1000 -or
        $dryPlan.shared_connection -ne $true -or
        $dryPlan.probe_serial_supplied -ne $true) {
        throw 'Dry-run metadata fields did not reflect the selected connection settings.'
    }

    Write-Output 'PASS fake CLI HOTPLUG upload allowlist, probe SN, 1000/4000 kHz, shared mode, DEV_CONNECT_ERR no-fallback, stderr capture, failure retention, and frozen-header guard'
    Write-Output "Success CSV: $successCsv"
}
finally {
    Remove-Item Env:RUNTIME_LOG_FAKE_BUFFER_PATH -ErrorAction SilentlyContinue
    Remove-Item Env:RUNTIME_LOG_FAKE_ARGS_PATH -ErrorAction SilentlyContinue
    Remove-Item Env:RUNTIME_LOG_FAKE_CALL_INDEX_PATH -ErrorAction SilentlyContinue
    Remove-Item Env:RUNTIME_LOG_FAKE_EXPECTED_ADDRESS -ErrorAction SilentlyContinue
    Remove-Item Env:RUNTIME_LOG_FAKE_EXPECTED_SN -ErrorAction SilentlyContinue
    Remove-Item Env:RUNTIME_LOG_FAKE_FREQ -ErrorAction SilentlyContinue
    Remove-Item Env:RUNTIME_LOG_FAKE_SHARED -ErrorAction SilentlyContinue
    Remove-Item Env:RUNTIME_LOG_FAKE_MODE -ErrorAction SilentlyContinue
}
