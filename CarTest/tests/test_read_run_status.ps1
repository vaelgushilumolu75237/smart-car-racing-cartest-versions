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
$attemptReadCount = 2 + [int][bool]$fixtureInfo.diagnostic_available +
    [int][bool]$fixtureInfo.speed_pi_diagnostic_available +
    [int][bool]$fixtureInfo.receive_diagnostic_available +
    [int][bool]$fixtureInfo.speed_pi_trace_available +
    3 * [int][bool]$fixtureInfo.ackermann_snapshot_available
$normalReadCount = 2 + $attemptReadCount
$normalFrequencies = @(); for ($i = 0; $i -lt $normalReadCount; ++$i) { $normalFrequencies += '1000' }
$fallbackFrequencies = @('1000'); for ($i = 0; $i -lt $normalReadCount; ++$i) { $fallbackFrequencies += '400' }
$descriptorFixture = Join-Path $fixtureRoot 'flash-rom-descriptor.bin'
$scalarFixture = Join-Path $fixtureRoot 'scalar-region.bin'
$headerFixture = Join-Path $fixtureRoot 'runtime-log-header.bin'
$serial = '1234567890'
$environmentNames = @(
    'RUN_STATUS_FAKE_ARGS_PATH', 'RUN_STATUS_FAKE_EXPECTED_SN',
    'RUN_STATUS_FAKE_DESCRIPTOR_ADDRESS', 'RUN_STATUS_FAKE_DESCRIPTOR_FIXTURE',
    'RUN_STATUS_FAKE_DIAGNOSTIC_ADDRESS', 'RUN_STATUS_FAKE_DIAGNOSTIC_FIXTURE',
    'RUN_STATUS_FAKE_SPEED_PI_DIAGNOSTIC_ADDRESS', 'RUN_STATUS_FAKE_SPEED_PI_DIAGNOSTIC_FIXTURE',
    'RUN_STATUS_FAKE_RECEIVE_DIAGNOSTIC_ADDRESS', 'RUN_STATUS_FAKE_RECEIVE_DIAGNOSTIC_FIXTURE',
    'RUN_STATUS_FAKE_SPEED_PI_TRACE_ADDRESS', 'RUN_STATUS_FAKE_SPEED_PI_TRACE_FIXTURE',
    'RUN_STATUS_FAKE_ACKERMANN_SNAPSHOT_ADDRESS', 'RUN_STATUS_FAKE_ACKERMANN_SNAPSHOT_FIXTURE',
    'RUN_STATUS_FAKE_ACKERMANN_GENERATION_ADDRESS', 'RUN_STATUS_FAKE_ACKERMANN_GENERATION_FIXTURE',
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
    $env:RUN_STATUS_FAKE_SPEED_PI_DIAGNOSTIC_ADDRESS = [string]$fixtureInfo.speed_pi_diagnostic_address
    $env:RUN_STATUS_FAKE_SPEED_PI_DIAGNOSTIC_FIXTURE = Join-Path $fixtureRoot 'speed-pi-diagnostic-idle.bin'
    $env:RUN_STATUS_FAKE_RECEIVE_DIAGNOSTIC_ADDRESS = [string]$fixtureInfo.receive_diagnostic_address
    $env:RUN_STATUS_FAKE_RECEIVE_DIAGNOSTIC_FIXTURE = [string]$fixtureInfo.receive_diagnostic_path
    $env:RUN_STATUS_FAKE_SPEED_PI_TRACE_ADDRESS = [string]$fixtureInfo.speed_pi_trace_address
    $env:RUN_STATUS_FAKE_SPEED_PI_TRACE_FIXTURE = [string]$fixtureInfo.speed_pi_trace_idle_path
    $env:RUN_STATUS_FAKE_ACKERMANN_SNAPSHOT_ADDRESS = [string]$fixtureInfo.ackermann_snapshot_address
    $env:RUN_STATUS_FAKE_ACKERMANN_SNAPSHOT_FIXTURE = [string]$fixtureInfo.ackermann_snapshot_path
    $env:RUN_STATUS_FAKE_ACKERMANN_GENERATION_ADDRESS = [string]$fixtureInfo.ackermann_generation_address
    $env:RUN_STATUS_FAKE_ACKERMANN_GENERATION_FIXTURE = [string]$fixtureInfo.ackermann_generation_path
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
    $expectedAddresses = New-Object System.Collections.Generic.List[string]
    $expectedSizes = New-Object System.Collections.Generic.List[int]
    $readIndex = 0
    if ($Frequencies.Count -gt 1 -and $Frequencies[0] -ne $Frequencies[1]) {
        $expectedAddresses.Add($descriptorAddress)
        $expectedSizes.Add(64)
        $readIndex++
    }
    $expectedAddresses.Add($descriptorAddress)
    $expectedSizes.Add(64)
    $expectedAddresses.Add($scalarAddress)
    $expectedSizes.Add([int]$fixtureInfo.scalar_size)
    $readIndex = $expectedAddresses.Count
    while ($readIndex -lt $Frequencies.Count) {
        $expectedAddresses.Add($headerAddress)
        $expectedSizes.Add(64)
        $readIndex++
        if ($fixtureInfo.diagnostic_available) {
            $expectedAddresses.Add([string]$fixtureInfo.diagnostic_address)
            $expectedSizes.Add(64)
            $readIndex++
        }
        if ($fixtureInfo.speed_pi_diagnostic_available) {
            $expectedAddresses.Add([string]$fixtureInfo.speed_pi_diagnostic_address)
            $expectedSizes.Add(128)
            $readIndex++
        }
        if ($fixtureInfo.receive_diagnostic_available) {
            $expectedAddresses.Add([string]$fixtureInfo.receive_diagnostic_address)
            $expectedSizes.Add(96)
            $readIndex++
        }
        if ($fixtureInfo.speed_pi_trace_available) {
            $expectedAddresses.Add([string]$fixtureInfo.speed_pi_trace_address)
            $expectedSizes.Add(64)
            $readIndex++
        }
        if ($fixtureInfo.ackermann_snapshot_available) {
            $expectedAddresses.Add([string]$fixtureInfo.ackermann_generation_address)
            $expectedSizes.Add(4)
            $expectedAddresses.Add([string]$fixtureInfo.ackermann_snapshot_address)
            $expectedSizes.Add(64)
            $expectedAddresses.Add([string]$fixtureInfo.ackermann_generation_address)
            $expectedSizes.Add(4)
            $readIndex += 3
        }
        $expectedAddresses.Add($headerAddress)
        $expectedSizes.Add(64)
        $readIndex++
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
    Assert-ReadSequence -ArgsLog $success.ArgsLog -Frequencies $normalFrequencies
    $successRun = Get-StatusRunDirectory $success.CaseRoot
    $status = Get-Content -LiteralPath (Join-Path $successRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($status.runtime_log_header.state_name -ne 'idle' -or
        $status.runtime_log_header.count -ne 0 -or $status.csv_created -or
        $status.reads_are_atomic_together -ne $false -or
        $status.flash_descriptor.status -ne 'matched' -or
        $status.connection.actual_frequency_khz -ne 950) {
        throw 'Status report did not preserve IDLE/no-CSV, non-atomic reads, descriptor, and actual rate.'
    }
    if ($fixtureInfo.receive_diagnostic_available -and
        ($status.receive_diagnostic_snapshot.accepted_millifps -ne 30000 -or
         $status.receive_diagnostic_snapshot.latest_sequence -ne 30)) {
        throw 'Receive snapshot did not preserve parser-accepted rate and sequence data.'
    }
    if ($fixtureInfo.speed_pi_trace_available -and
        $status.speed_pi_trace_header.state_name -ne 'idle') {
        throw 'IDLE status did not report an IDLE PI trace header.'
    }
    if ($fixtureInfo.ackermann_snapshot_available -and
        ($status.ackermann_snapshot.status -ne 'frozen' -or
         $status.ackermann_snapshot.generation -ne 2 -or
         $status.reads.ackermann_snapshot.generation_read.size -ne 4)) {
        throw 'ACK1 snapshot/guard reads were not decoded as a stable frozen coordination snapshot.'
    }
    if ($fixtureInfo.ackermann_snapshot_available -and
        $success.Output -notmatch 'ACK1=frozen/none') {
        throw 'Status wrapper output did not expose the ACK1 frozen/error classification.'
    }

    $badCrcHeader = Join-Path $fixtureRoot 'runtime-log-header-idle-bad-crc.bin'
    $diagnosticIdle = Join-Path $fixtureRoot 'runtime-diagnostic.bin'
    $retrySequence = @($descriptorFixture, $scalarFixture, $badCrcHeader, $diagnosticIdle)
    if ($fixtureInfo.speed_pi_diagnostic_available) {
        $retrySequence += Join-Path $fixtureRoot 'speed-pi-diagnostic-idle.bin'
    }
    if ($fixtureInfo.receive_diagnostic_available) { $retrySequence += $fixtureInfo.receive_diagnostic_path }
    if ($fixtureInfo.speed_pi_trace_available) { $retrySequence += $fixtureInfo.speed_pi_trace_idle_path }
    if ($fixtureInfo.ackermann_snapshot_available) {
        $retrySequence += @($fixtureInfo.ackermann_generation_path,
            $fixtureInfo.ackermann_snapshot_path, $fixtureInfo.ackermann_generation_path)
    }
    $retrySequence += $badCrcHeader
    $retrySequence += @($headerFixture, $diagnosticIdle)
    if ($fixtureInfo.speed_pi_diagnostic_available) {
        $retrySequence += Join-Path $fixtureRoot 'speed-pi-diagnostic-idle.bin'
    }
    if ($fixtureInfo.receive_diagnostic_available) { $retrySequence += $fixtureInfo.receive_diagnostic_path }
    if ($fixtureInfo.speed_pi_trace_available) { $retrySequence += $fixtureInfo.speed_pi_trace_idle_path }
    if ($fixtureInfo.ackermann_snapshot_available) {
        $retrySequence += @($fixtureInfo.ackermann_generation_path,
            $fixtureInfo.ackermann_snapshot_path, $fixtureInfo.ackermann_generation_path)
    }
    $retrySequence += $headerFixture
    $crcRetry = Invoke-StatusCase -Name 'retry-invalid-header-crc-then-idle' `
        -Mode success -UploadFixtureSequence $retrySequence
    if ($crcRetry.ExitCode -ne 0) { throw "CRC retry status failed: $($crcRetry.Output)" }
    $crcRetryRun = Get-StatusRunDirectory $crcRetry.CaseRoot
    $crcRetryStatus = Get-Content -LiteralPath (Join-Path $crcRetryRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $crcRetryLog = Get-Content -LiteralPath (Join-Path $crcRetryRun.FullName 'snapshot-retries.log') -Raw -Encoding UTF8
    if ($crcRetryStatus.status_class -ne 'neverstarted' -or $crcRetryStatus.csv_created -or
        @(Get-Content -LiteralPath $crcRetry.ArgsLog).Count -ne (2 + 2 * $attemptReadCount) -or
        $crcRetryLog -notmatch 'Attempt 1:') {
        throw 'CRC-invalid bracket was not retried before accepting IDLE without creating a CSV.'
    }

    $diagnosticFrozen = Join-Path $fixtureRoot 'runtime-diagnostic-frozen.bin'
    $frozenHeader = Join-Path $fixtureRoot 'runtime-log-header-frozen.bin'
    $inconsistentSequence = @($descriptorFixture, $scalarFixture, $headerFixture,
        $diagnosticFrozen)
    if ($fixtureInfo.speed_pi_diagnostic_available) {
        $inconsistentSequence += Join-Path $fixtureRoot 'speed-pi-diagnostic-frozen.bin'
    }
    if ($fixtureInfo.receive_diagnostic_available) { $inconsistentSequence += $fixtureInfo.receive_diagnostic_path }
    if ($fixtureInfo.speed_pi_trace_available) { $inconsistentSequence += $fixtureInfo.speed_pi_trace_frozen_header_path }
    if ($fixtureInfo.ackermann_snapshot_available) {
        $inconsistentSequence += @($fixtureInfo.ackermann_generation_path,
            $fixtureInfo.ackermann_snapshot_path, $fixtureInfo.ackermann_generation_path)
    }
    $inconsistentSequence += @($frozenHeader, $frozenHeader, $diagnosticFrozen)
    if ($fixtureInfo.speed_pi_diagnostic_available) {
        $inconsistentSequence += Join-Path $fixtureRoot 'speed-pi-diagnostic-frozen.bin'
    }
    if ($fixtureInfo.receive_diagnostic_available) { $inconsistentSequence += $fixtureInfo.receive_diagnostic_path }
    if ($fixtureInfo.speed_pi_trace_available) { $inconsistentSequence += $fixtureInfo.speed_pi_trace_frozen_header_path }
    if ($fixtureInfo.ackermann_snapshot_available) {
        $inconsistentSequence += @($fixtureInfo.ackermann_generation_path,
            $fixtureInfo.ackermann_snapshot_path, $fixtureInfo.ackermann_generation_path)
    }
    $inconsistentSequence += $frozenHeader
    $inconsistentRetry = Invoke-StatusCase -Name 'retry-transition-then-stable-frozen' `
        -Mode success -UploadFixtureSequence $inconsistentSequence
    if ($inconsistentRetry.ExitCode -ne 0) { throw "Inconsistent retry status failed: $($inconsistentRetry.Output)" }
    $inconsistentRun = Get-StatusRunDirectory $inconsistentRetry.CaseRoot
    $inconsistentStatus = Get-Content -LiteralPath (Join-Path $inconsistentRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $inconsistentLog = Get-Content -LiteralPath (Join-Path $inconsistentRun.FullName 'snapshot-retries.log') -Raw -Encoding UTF8
    if ($inconsistentStatus.status_class -ne 'frozen' -or
        $inconsistentStatus.runtime_log_header.count -ne $inconsistentStatus.runtime_log_header_after.count -or
        $inconsistentStatus.runtime_diagnostic_snapshot.cpu_header_state_name -ne 'frozen' -or
        @(Get-Content -LiteralPath $inconsistentRetry.ArgsLog).Count -ne (2 + 2 * $attemptReadCount) -or
        $inconsistentLog -notmatch 'Attempt 1:') {
        throw 'IDLE-to-FROZEN bracket was not retried into an exact stable FROZEN match.'
    }

    $exclusive = Invoke-StatusCase -Name 'legacy-string-false-works-in-powershell-child' -Mode success -Shared 'false'
    if ($exclusive.ExitCode -ne 0) { throw "String-bool false did not bind in PowerShell child: $($exclusive.Output)" }
    Assert-ReadSequence -ArgsLog $exclusive.ArgsLog -Frequencies $normalFrequencies -Shared:$false

    $fallback = Invoke-StatusCase -Name 'fallback-1000-to-400' -Mode 'connect-error-1000' -Frequency '1000' -Shared 'true' -ActualFrequency '375'
    if ($fallback.ExitCode -ne 0) { throw "Status connection fallback failed: $($fallback.Output)" }
    Assert-ReadSequence -ArgsLog $fallback.ArgsLog -Frequencies $fallbackFrequencies
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
        $dryJson.reads.Count -ne (4 + [int][bool]$fixtureInfo.diagnostic_available +
            [int][bool]$fixtureInfo.speed_pi_diagnostic_available +
            [int][bool]$fixtureInfo.receive_diagnostic_available +
            [int][bool]$fixtureInfo.speed_pi_trace_available +
            3 * [int][bool]$fixtureInfo.ackermann_snapshot_available) -or
        (Test-Path -LiteralPath $dry.ArgsLog)) {
        throw 'Status DryRun must list Flash/scalar/header/diagnostic/header reads without running the CLI.'
    }
    if (@($dryJson.reads[0].arguments)[0] -ne '-c' -or
        @($dryJson.reads[0].arguments)[3] -ne 'freq=1000' -or
        @($dryJson.reads[0].arguments)[4] -ne 'shared' -or
        @($dryJson.reads[0].arguments)[6] -ne '--upload') {
        throw 'Status DryRun did not use the shared 1000 kHz Flash-first plan.'
    }
    $dryReadIndex = 3
    if ($fixtureInfo.diagnostic_available) {
        if (@($dryJson.reads[$dryReadIndex].arguments)[7] -ne [string]$fixtureInfo.diagnostic_address -or
            @($dryJson.reads[$dryReadIndex].arguments)[8] -ne '64') {
            throw 'Status DryRun omitted the 64-byte runtime diagnostic snapshot.'
        }
        $dryReadIndex++
    }
    if ($fixtureInfo.speed_pi_diagnostic_available) {
        if (@($dryJson.reads[$dryReadIndex].arguments)[7] -ne [string]$fixtureInfo.speed_pi_diagnostic_address -or
            @($dryJson.reads[$dryReadIndex].arguments)[8] -ne '128') {
            throw 'Status DryRun omitted the 128-byte speed PI diagnostic snapshot.'
        }
        $dryReadIndex++
    }
    if ($fixtureInfo.receive_diagnostic_available) {
        if (@($dryJson.reads[$dryReadIndex].arguments)[7] -ne [string]$fixtureInfo.receive_diagnostic_address -or
            @($dryJson.reads[$dryReadIndex].arguments)[8] -ne '96') {
            throw 'Status DryRun omitted the 96-byte receive-rate and sequence snapshot.'
        }
        $dryReadIndex++
    }
    if ($fixtureInfo.speed_pi_trace_available) {
        if (@($dryJson.reads[$dryReadIndex].arguments)[7] -ne [string]$fixtureInfo.speed_pi_trace_address -or
            @($dryJson.reads[$dryReadIndex].arguments)[8] -ne '64') {
            throw 'Status DryRun omitted the 64-byte PI trace header snapshot.'
        }
        $dryReadIndex++
    }
    if ($fixtureInfo.ackermann_snapshot_available) {
        $ackRead = @($dryJson.reads[$dryReadIndex].arguments)
        if ($ackRead[7] -ne [string]$fixtureInfo.ackermann_generation_address -or
            $ackRead[8] -ne '4') {
            throw 'Status DryRun omitted the leading 4-byte ACK1 generation guard.'
        }
        $dryReadIndex++
        $ackRead = @($dryJson.reads[$dryReadIndex].arguments)
        if ($ackRead[7] -ne [string]$fixtureInfo.ackermann_snapshot_address -or
            $ackRead[8] -ne '64') {
            throw 'Status DryRun omitted the 64-byte ACK1 coordination snapshot.'
        }
        $dryReadIndex++
        $ackRead = @($dryJson.reads[$dryReadIndex].arguments)
        if ($ackRead[7] -ne [string]$fixtureInfo.ackermann_generation_address -or
            $ackRead[8] -ne '4') {
            throw 'Status DryRun omitted the trailing 4-byte ACK1 generation guard.'
        }
        $dryReadIndex++
    }
    if (@($dryJson.reads[$dryReadIndex].arguments)[7] -ne [string]$fixtureInfo.header_address -or
        @($dryJson.reads[$dryReadIndex].arguments)[8] -ne '64') {
        throw 'Status DryRun did not finish the header bracket after its diagnostic snapshots.'
    }

    Write-Output 'PASS status child-process shared string true/false, IDLE no-CSV, ACK1 before/snapshot/after generation reads and CRC decode, CRC-fail-then-pass and inconsistent-then-pass retries, stable FROZEN, Flash-first identity, 1000-to-400 fallback, actual frequency, non-atomic warning, and DryRun'
    Write-Output "Evidence: $OutputRoot"
}
finally {
    foreach ($name in $environmentNames) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name])
    }
}
