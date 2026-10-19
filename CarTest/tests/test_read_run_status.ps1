[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Elf,
    [Parameter(Mandatory = $true)][string]$HeaderFixture,
    [Parameter(Mandatory = $true)][string]$ScalarFixture,
    [Parameter(Mandatory = $true)][string]$OutputRoot
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$wrapper = Join-Path $projectRoot 'tools\read_run_status.ps1'
$fakeSource = Join-Path $PSScriptRoot 'support\FakeRunStatusCubeProgrammer.cs'
$fixtureBuilder = Join-Path $PSScriptRoot 'support\make_status_fixtures.py'
$python = (Get-Command -Name 'python.exe' -ErrorAction Stop).Source
$powershellExe = Join-Path $PSHOME 'powershell.exe'
$resolvedElf = (Resolve-Path -LiteralPath $Elf).Path
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
New-Item -ItemType Directory -Path $OutputRoot -Force | Out-Null
$fakeDirectory = Join-Path $OutputRoot 'fake status cli with spaces'
New-Item -ItemType Directory -Path $fakeDirectory -Force | Out-Null
$fakeExe = Join-Path $fakeDirectory 'STM32_Programmer_CLI.exe'
Add-Type -TypeDefinition (Get-Content -LiteralPath $fakeSource -Raw -Encoding UTF8) `
    -OutputAssembly $fakeExe -OutputType ConsoleApplication

$fixtureRoot = Join-Path $OutputRoot 'target-matched synthetic fixtures'
$fixtureBuilderOutput = @(& $python $fixtureBuilder --elf $resolvedElf --out $fixtureRoot)
if ($LASTEXITCODE -ne 0) { throw 'Could not synthesize matching ELF status fixtures.' }
$fixtureInfo = ($fixtureBuilderOutput -join [Environment]::NewLine) | ConvertFrom-Json
$descriptorFixture = Join-Path $fixtureRoot 'flash-rom-descriptor.bin'
$scalarFixture = Join-Path $fixtureRoot 'scalar-region.bin'
$headerFixture = Join-Path $fixtureRoot 'runtime-log-header.bin'
$serial = '1234567890'
$environmentNames = @(
    'RUN_STATUS_FAKE_ARGS_PATH', 'RUN_STATUS_FAKE_EXPECTED_SN',
    'RUN_STATUS_FAKE_DESCRIPTOR_ADDRESS', 'RUN_STATUS_FAKE_DESCRIPTOR_FIXTURE',
    'RUN_STATUS_FAKE_DIAGNOSTIC_ADDRESS', 'RUN_STATUS_FAKE_DIAGNOSTIC_FIXTURE',
    'RUN_STATUS_FAKE_SCALAR_ADDRESS', 'RUN_STATUS_FAKE_SCALAR_SIZE',
    'RUN_STATUS_FAKE_SCALAR_FIXTURE', 'RUN_STATUS_FAKE_HEADER_ADDRESS',
    'RUN_STATUS_FAKE_HEADER_FIXTURE', 'RUN_STATUS_FAKE_MODE',
    'RUN_STATUS_FAKE_FREQ', 'RUN_STATUS_FAKE_SHARED', 'RUN_STATUS_FAKE_PROBES',
    'RUN_STATUS_FAKE_ACTUAL_FREQ', 'RUN_STATUS_FAKE_CALL_INDEX_PATH',
    'RUN_STATUS_FAKE_UPLOAD_SEQUENCE'
)
$savedEnvironment = @{}
foreach ($name in $environmentNames) { $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name) }

function Invoke-StatusCase {
    param(
        [string]$Name,
        [string]$Mode = 'success',
        [string]$Frequency = '1000',
        [string]$Shared = 'true',
        [string]$ActualFrequency = '950',
        [switch]$DryRun,
        [string]$DescriptorPath = $descriptorFixture,
        [string[]]$UploadFixtureSequence = @()
    )
    $caseRoot = Join-Path $OutputRoot $Name
    New-Item -ItemType Directory -Path $caseRoot -Force | Out-Null
    $argsLog = Join-Path $caseRoot 'fake-cli-args.tsv'
    $env:RUN_STATUS_FAKE_ARGS_PATH = $argsLog
    $env:RUN_STATUS_FAKE_EXPECTED_SN = $serial
    $env:RUN_STATUS_FAKE_DESCRIPTOR_ADDRESS = [string]$fixtureInfo.descriptor_address
    $env:RUN_STATUS_FAKE_DESCRIPTOR_FIXTURE = $DescriptorPath
    $env:RUN_STATUS_FAKE_DIAGNOSTIC_ADDRESS = [string]$fixtureInfo.diagnostic_address
    $env:RUN_STATUS_FAKE_DIAGNOSTIC_FIXTURE = Join-Path $fixtureRoot 'runtime-diagnostic.bin'
    $env:RUN_STATUS_FAKE_SCALAR_ADDRESS = [string]$fixtureInfo.scalar_address
    $env:RUN_STATUS_FAKE_SCALAR_SIZE = [string]$fixtureInfo.scalar_size
    $env:RUN_STATUS_FAKE_SCALAR_FIXTURE = $scalarFixture
    $env:RUN_STATUS_FAKE_HEADER_ADDRESS = [string]$fixtureInfo.header_address
    $env:RUN_STATUS_FAKE_HEADER_FIXTURE = $headerFixture
    $env:RUN_STATUS_FAKE_MODE = $Mode
    $env:RUN_STATUS_FAKE_FREQ = $Frequency
    $env:RUN_STATUS_FAKE_SHARED = if ($Shared -match '^(true|1)$') { '1' } else { '0' }
    $env:RUN_STATUS_FAKE_PROBES = $serial
    $env:RUN_STATUS_FAKE_ACTUAL_FREQ = $ActualFrequency
    $env:RUN_STATUS_FAKE_CALL_INDEX_PATH = Join-Path $caseRoot 'fake-cli-call-index.txt'
    $env:RUN_STATUS_FAKE_UPLOAD_SEQUENCE = $UploadFixtureSequence -join '|'

    $arguments = @('-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
        '-File', $wrapper, '-Elf', $resolvedElf,
        '-OutDirectory', (Join-Path $caseRoot 'output'),
        '-ProgrammerCLI', $fakeExe, '-ProbeSN', $serial,
        '-ConnectionFrequencyKhz', $Frequency, '-SharedConnection', $Shared)
    if ($DryRun) { $arguments += '-DryRun' }
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = @(& $powershellExe @arguments 2>&1)
        $exitCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previousPreference }
    $text = (($output | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine)
    Set-Content -LiteralPath (Join-Path $caseRoot 'wrapper-output.log') -Value $text -Encoding UTF8
    return [PSCustomObject]@{ CaseRoot = $caseRoot; ArgsLog = $argsLog; ExitCode = $exitCode; Output = $text }
}

function Get-StatusRunDirectory {
    param([string]$CaseRoot)
    $root = Join-Path $CaseRoot 'output'
    if (-not (Test-Path -LiteralPath $root -PathType Container)) { return $null }
    return Get-ChildItem -LiteralPath $root -Directory | Sort-Object LastWriteTime -Descending | Select-Object -First 1
}

function Assert-ReadSequence {
    param([string]$ArgsLog, [string[]]$Frequencies, [bool]$Shared = $true)
    $lines = @(Get-Content -LiteralPath $ArgsLog -Encoding UTF8)
    if ($lines.Count -ne $Frequencies.Count) { throw "Expected $($Frequencies.Count) reads, got $($lines.Count)." }
    $descriptorAddress = [string]$fixtureInfo.descriptor_address
    $scalarAddress = [string]$fixtureInfo.scalar_address
    $headerAddress = [string]$fixtureInfo.header_address
    if ($Frequencies.Count -eq 6) {
        $expectedAddresses = @($descriptorAddress, $descriptorAddress, $scalarAddress, $headerAddress)
        $expectedAddresses += @([string]$fixtureInfo.diagnostic_address, $headerAddress)
        $expectedSizes = @(64, 64, [int]$fixtureInfo.scalar_size, 64, 64, 64)
    } elseif ($Frequencies.Count -eq 5) {
        $expectedAddresses = @($descriptorAddress, $scalarAddress, $headerAddress,
            [string]$fixtureInfo.diagnostic_address, $headerAddress)
        $expectedSizes = @(64, [int]$fixtureInfo.scalar_size, 64, 64, 64)
    } elseif ($Frequencies.Count -eq 4) {
        $expectedAddresses = @($descriptorAddress, $descriptorAddress, $scalarAddress, $headerAddress)
        $expectedSizes = @(64, 64, [int]$fixtureInfo.scalar_size, 64)
    } else {
        $expectedAddresses = @($descriptorAddress, $scalarAddress, $headerAddress)
        $expectedSizes = @(64, [int]$fixtureInfo.scalar_size, 64)
    }
    for ($i = 0; $i -lt $lines.Count; ++$i) {
        $tokens = $lines[$i].Split([char]9)
        $offset = 4
        if ($Shared) {
            if ($tokens[$offset] -ne 'shared') { throw "Read $i did not use shared mode." }
            $offset++
        } elseif ($tokens[$offset] -eq 'shared') {
            throw "Read $i unexpectedly used shared mode."
        }
        if ($tokens.Count -ne $offset + 5 -or $tokens[0] -ne '-c' -or
            $tokens[1] -ne 'port=SWD' -or $tokens[2] -ne 'mode=HOTPLUG' -or
            $tokens[3] -ne ("freq={0}" -f $Frequencies[$i]) -or
            $tokens[$offset] -ne "sn=$serial" -or $tokens[$offset + 1] -ne '--upload' -or
            $tokens[$offset + 2] -ne $expectedAddresses[$i] -or
            [int]$tokens[$offset + 3] -ne $expectedSizes[$i]) {
            throw "Unexpected bounded read ${i}: $($lines[$i])"
        }
    }
}

try {
    $success = Invoke-StatusCase -Name 'IDLE-status-with-shared-string-true' -Mode success -Shared 'true'
    if ($success.ExitCode -ne 0) { throw "Target-matched IDLE status failed: $($success.Output)" }
    Assert-ReadSequence -ArgsLog $success.ArgsLog -Frequencies @('1000', '1000', '1000', '1000', '1000')
    $successRun = Get-StatusRunDirectory $success.CaseRoot
    $status = Get-Content -LiteralPath (Join-Path $successRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($status.runtime_log_header.state_name -ne 'idle' -or
        $status.runtime_log_header.count -ne 0 -or $status.csv_created -or
        $status.reads_are_atomic_together -ne $false -or
        $status.flash_descriptor.status -ne 'matched' -or
        $status.connection.actual_frequency_khz -ne 950) {
        throw 'Status report did not preserve IDLE/no-CSV, non-atomic reads, descriptor, and actual rate.'
    }

    $badCrcHeader = Join-Path $fixtureRoot 'runtime-log-header-idle-bad-crc.bin'
    $diagnosticIdle = Join-Path $fixtureRoot 'runtime-diagnostic.bin'
    $retrySequence = @($descriptorFixture, $scalarFixture, $badCrcHeader, $diagnosticIdle,
        $badCrcHeader, $headerFixture, $diagnosticIdle, $headerFixture)
    $crcRetry = Invoke-StatusCase -Name 'retry-invalid-header-crc-then-idle' `
        -Mode success -UploadFixtureSequence $retrySequence
    if ($crcRetry.ExitCode -ne 0) { throw "CRC retry status failed: $($crcRetry.Output)" }
    $crcRetryRun = Get-StatusRunDirectory $crcRetry.CaseRoot
    $crcRetryStatus = Get-Content -LiteralPath (Join-Path $crcRetryRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $crcRetryLog = Get-Content -LiteralPath (Join-Path $crcRetryRun.FullName 'snapshot-retries.log') -Raw -Encoding UTF8
    if ($crcRetryStatus.status_class -ne 'neverstarted' -or $crcRetryStatus.csv_created -or
        @(Get-Content -LiteralPath $crcRetry.ArgsLog).Count -ne 8 -or
        $crcRetryLog -notmatch 'Attempt 1:') {
        throw 'CRC-invalid bracket was not retried before accepting IDLE without creating a CSV.'
    }

    $diagnosticFrozen = Join-Path $fixtureRoot 'runtime-diagnostic-frozen.bin'
    $frozenHeader = Join-Path $fixtureRoot 'runtime-log-header-frozen.bin'
    $inconsistentSequence = @($descriptorFixture, $scalarFixture, $headerFixture,
        $diagnosticFrozen, $frozenHeader, $frozenHeader, $diagnosticFrozen, $frozenHeader)
    $inconsistentRetry = Invoke-StatusCase -Name 'retry-transition-then-stable-frozen' `
        -Mode success -UploadFixtureSequence $inconsistentSequence
    if ($inconsistentRetry.ExitCode -ne 0) { throw "Inconsistent retry status failed: $($inconsistentRetry.Output)" }
    $inconsistentRun = Get-StatusRunDirectory $inconsistentRetry.CaseRoot
    $inconsistentStatus = Get-Content -LiteralPath (Join-Path $inconsistentRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $inconsistentLog = Get-Content -LiteralPath (Join-Path $inconsistentRun.FullName 'snapshot-retries.log') -Raw -Encoding UTF8
    if ($inconsistentStatus.status_class -ne 'frozen' -or
        $inconsistentStatus.runtime_log_header.count -ne $inconsistentStatus.runtime_log_header_after.count -or
        $inconsistentStatus.runtime_diagnostic_snapshot.cpu_header_state_name -ne 'frozen' -or
        @(Get-Content -LiteralPath $inconsistentRetry.ArgsLog).Count -ne 8 -or
        $inconsistentLog -notmatch 'Attempt 1:') {
        throw 'IDLE-to-FROZEN bracket was not retried into an exact stable FROZEN match.'
    }

    $exclusive = Invoke-StatusCase -Name 'legacy-string-false-works-in-powershell-child' -Mode success -Shared 'false'
    if ($exclusive.ExitCode -ne 0) { throw "String-bool false did not bind in PowerShell child: $($exclusive.Output)" }
    Assert-ReadSequence -ArgsLog $exclusive.ArgsLog -Frequencies @('1000', '1000', '1000', '1000', '1000') -Shared:$false

    $fallback = Invoke-StatusCase -Name 'fallback-1000-to-400' -Mode 'connect-error-1000' -Frequency '1000' -Shared 'true' -ActualFrequency '375'
    if ($fallback.ExitCode -ne 0) { throw "Status connection fallback failed: $($fallback.Output)" }
    Assert-ReadSequence -ArgsLog $fallback.ArgsLog -Frequencies @('1000', '400', '400', '400', '400', '400')
    $fallbackRun = Get-StatusRunDirectory $fallback.CaseRoot
    $fallbackStatus = Get-Content -LiteralPath (Join-Path $fallbackRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($fallbackStatus.connection.frequency_khz -ne 400 -or
        $fallbackStatus.connection.actual_frequency_khz -ne 375 -or
        $fallbackStatus.connection.attempts[0].error_class -ne 'core_inaccessible') {
        throw 'Status fallback did not reuse the successful rate or retain the failed attempt.'
    }

    $wrongDescriptor = Join-Path $OutputRoot 'wrong-status-descriptor.bin'
    $bad = [IO.File]::ReadAllBytes($descriptorFixture)
    $bad[28] = $bad[28] -bxor 1
    [IO.File]::WriteAllBytes($wrongDescriptor, $bad)
    $mismatch = Invoke-StatusCase -Name 'wrong-elf-no-ram-read' -Mode success -DescriptorPath $wrongDescriptor
    if ($mismatch.ExitCode -eq 0) { throw 'Wrong target Flash descriptor unexpectedly passed status read.' }
    $mismatchRun = Get-StatusRunDirectory $mismatch.CaseRoot
    $mismatchStatus = Get-Content -LiteralPath (Join-Path $mismatchRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($mismatchStatus.error_class -ne 'elf_mismatch' -or
        @(Get-Content -LiteralPath $mismatch.ArgsLog).Count -ne 1 -or
        (Test-Path -LiteralPath (Join-Path $mismatchRun.FullName 'scalar-region.bin'))) {
        throw 'Flash descriptor mismatch was not fail-fast before RAM scalar reads.'
    }

    $dry = Invoke-StatusCase -Name 'dry-run-no-target-command' -Mode success -DryRun
    if ($dry.ExitCode -ne 0) { throw "Status DryRun failed: $($dry.Output)" }
    $dryJson = $dry.Output | ConvertFrom-Json
    if ($dryJson.status -ne 'dry_run' -or $dryJson.connected -or
        $dryJson.reads.Count -ne 5 -or (Test-Path -LiteralPath $dry.ArgsLog)) {
        throw 'Status DryRun must list Flash/scalar/header/diagnostic/header reads without running the CLI.'
    }
    if (@($dryJson.reads[0].arguments)[0] -ne '-c' -or
        @($dryJson.reads[0].arguments)[3] -ne 'freq=1000' -or
        @($dryJson.reads[0].arguments)[4] -ne 'shared' -or
        @($dryJson.reads[0].arguments)[6] -ne '--upload') {
        throw 'Status DryRun did not use the shared 1000 kHz Flash-first plan.'
    }
    if (@($dryJson.reads[3].arguments)[7] -ne [string]$fixtureInfo.diagnostic_address -or
        @($dryJson.reads[3].arguments)[8] -ne '64' -or
        @($dryJson.reads[4].arguments)[7] -ne [string]$fixtureInfo.header_address -or
        @($dryJson.reads[4].arguments)[8] -ne '64') {
        throw 'Status DryRun did not bracket the 64-byte diagnostic snapshot with runtime-log headers.'
    }

    Write-Output 'PASS status child-process shared string true/false, IDLE no-CSV, CRC-fail-then-pass and inconsistent-then-pass retries, stable FROZEN, Flash-first identity, 1000-to-400 fallback, actual frequency, non-atomic warning, and DryRun'
    Write-Output "Evidence: $OutputRoot"
}
finally {
    foreach ($name in $environmentNames) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name])
    }
}
