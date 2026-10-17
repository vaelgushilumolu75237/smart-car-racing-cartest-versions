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
$statusReader = Join-Path $projectRoot 'tools\run_status_reader.py'
$fakeSource = Join-Path $PSScriptRoot 'support\FakeCubeProgrammer.cs'
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
New-Item -ItemType Directory -Path $OutputRoot -Force | Out-Null
$fakeDirectory = Join-Path $OutputRoot 'fake cli with spaces'
New-Item -ItemType Directory -Path $fakeDirectory -Force | Out-Null
$fakeExe = Join-Path $fakeDirectory 'STM32_Programmer_CLI.exe'
Add-Type -TypeDefinition (Get-Content -LiteralPath $fakeSource -Raw -Encoding UTF8) `
    -OutputAssembly $fakeExe -OutputType ConsoleApplication

$python = (Get-Command -Name 'python.exe' -ErrorAction Stop).Source
$powershellExe = Join-Path $PSHOME 'powershell.exe'
$resolvedElf = (Resolve-Path -LiteralPath $Elf).Path
$inspect = @(& $python $validator inspect --elf $resolvedElf)
if ($LASTEXITCODE -ne 0) { throw 'Could not inspect the selected ELF for fake CLI tests.' }
$layout = ($inspect -join [Environment]::NewLine) | ConvertFrom-Json
$statusPlanLines = @(& $python $statusReader plan --elf $resolvedElf)
if ($LASTEXITCODE -ne 0) { throw 'Could not inspect status read plan for fake CLI tests.' }
$statusPlan = ($statusPlanLines -join [Environment]::NewLine) | ConvertFrom-Json
$serial = '1234567890'

$descriptorFixture = Join-Path $OutputRoot 'target-flash-descriptor.bin'
$descriptorBytes = New-Object byte[] 64
for ($i = 0; $i -lt 64; ++$i) {
    $descriptorBytes[$i] = [Convert]::ToByte($layout.descriptor_bytes_hex.Substring($i * 2, 2), 16)
}
[IO.File]::WriteAllBytes($descriptorFixture, $descriptorBytes)
$scalarFixture = Join-Path $OutputRoot 'status-scalar-region.bin'
[IO.File]::WriteAllBytes($scalarFixture, (New-Object byte[] ([int]$statusPlan.scalar_read.size)))

$fixtureBytes = [IO.File]::ReadAllBytes((Resolve-Path -LiteralPath $Fixture).Path)
$fixtureHeaderPath = Join-Path $OutputRoot 'fixture-header.bin'
$fixtureHeaderBytes = New-Object byte[] 64
[Array]::Copy($fixtureBytes, 0, $fixtureHeaderBytes, 0, 64)
[IO.File]::WriteAllBytes($fixtureHeaderPath, $fixtureHeaderBytes)
$fixtureHeaderJson = @(& $python $validator check-header --elf $resolvedElf --header $fixtureHeaderPath)
if ($LASTEXITCODE -ne 0) { throw 'The synthetic logger fixture header does not match the selected ELF.' }
$fixtureHeader = ($fixtureHeaderJson -join [Environment]::NewLine) | ConvertFrom-Json

$environmentNames = @(
    'RUNTIME_LOG_FAKE_BUFFER_PATH', 'RUNTIME_LOG_FAKE_ARGS_PATH',
    'RUNTIME_LOG_FAKE_CALL_INDEX_PATH', 'RUNTIME_LOG_FAKE_EXPECTED_ADDRESS',
    'RUNTIME_LOG_FAKE_EXPECTED_SN', 'RUNTIME_LOG_FAKE_FREQ',
    'RUNTIME_LOG_FAKE_SHARED', 'RUNTIME_LOG_FAKE_MODE',
    'RUNTIME_LOG_FAKE_DESCRIPTOR_ADDRESS', 'RUNTIME_LOG_FAKE_DESCRIPTOR_PATH',
    'RUNTIME_LOG_FAKE_STATUS_SCALAR_ADDRESS', 'RUNTIME_LOG_FAKE_STATUS_SCALAR_PATH',
    'RUNTIME_LOG_FAKE_STATUS_SCALAR_SIZE', 'RUNTIME_LOG_FAKE_PROBES',
    'RUNTIME_LOG_FAKE_ACTUAL_FREQ'
)
$savedEnvironment = @{}
foreach ($name in $environmentNames) { $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name) }

function Invoke-FakeExport {
    param(
        [string]$Name,
        [string]$Mode = 'success',
        [string]$Frequency = '1000',
        [bool]$Shared = $true,
        [string]$ActualFrequency = '950',
        [bool]$PassProbeSN = $true,
        [string]$Probes = $serial,
        [string]$DescriptorPath = $descriptorFixture
    )
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
    $env:RUNTIME_LOG_FAKE_DESCRIPTOR_ADDRESS = [string]$layout.descriptor_address
    $env:RUNTIME_LOG_FAKE_DESCRIPTOR_PATH = $DescriptorPath
    $env:RUNTIME_LOG_FAKE_STATUS_SCALAR_ADDRESS = [string]$statusPlan.scalar_read.address
    $env:RUNTIME_LOG_FAKE_STATUS_SCALAR_PATH = $scalarFixture
    $env:RUNTIME_LOG_FAKE_STATUS_SCALAR_SIZE = [string]$statusPlan.scalar_read.size
    $env:RUNTIME_LOG_FAKE_PROBES = $Probes
    $env:RUNTIME_LOG_FAKE_ACTUAL_FREQ = $ActualFrequency

    $childArguments = @(
        '-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
        '-File', $exportScript,
        '-Elf', $resolvedElf,
        '-OutDirectory', (Join-Path $caseRoot 'out x'),
        '-ProgrammerCLI', $fakeExe,
        '-ConnectionFrequencyKhz', $Frequency
    )
    if ($PassProbeSN) { $childArguments += @('-ProbeSN', $serial) }
    if ($Shared) { $childArguments += '-SharedConnection' }
    else { $childArguments += '-SharedConnection:$false' }
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $childOutput = @(& $powershellExe @childArguments 2>&1)
        $exitCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previousPreference }
    $text = (($childOutput | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine)
    Set-Content -LiteralPath (Join-Path $caseRoot 'wrapper-output.log') -Value $text -Encoding UTF8
    return [PSCustomObject]@{ CaseRoot = $caseRoot; ArgsLog = $argsLog; ExitCode = $exitCode; Output = $text }
}

function Get-ExportRunDirectory {
    param([string]$CaseRoot)
    return Get-ChildItem -LiteralPath (Join-Path $CaseRoot 'out x') -Directory |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1
}

function Assert-HotPlugUploadArguments {
    param(
        [string]$ArgsLog,
        [string[]]$ExpectedFrequencies,
        [bool]$Shared = $true,
        [int[]]$ExpectedSizes = @(64, 64, [int]$fixtureHeader.payload_size, 64, 64, [int]$statusPlan.scalar_read.size, 64)
    )
    $lines = @(Get-Content -LiteralPath $ArgsLog -Encoding UTF8)
    if ($ExpectedFrequencies.Count -eq 8 -and $ExpectedSizes.Count -eq 7) {
        $ExpectedSizes = @(64) + $ExpectedSizes
    }
    if ($lines.Count -ne $ExpectedFrequencies.Count -or $lines.Count -ne $ExpectedSizes.Count) {
        throw "Expected $($ExpectedFrequencies.Count) uploads, found $($lines.Count)."
    }
    $expectedAddresses = @(
        [string]$layout.descriptor_address,
        [string]$layout.storage_address,
        [string]$layout.storage_address,
        [string]$layout.storage_address,
        [string]$layout.descriptor_address,
        [string]$statusPlan.scalar_read.address,
        [string]$statusPlan.header_read.address
    )
    if ($ExpectedFrequencies.Count -eq 8) {
        $expectedAddresses = @([string]$layout.descriptor_address) + $expectedAddresses
    }
    $forbidden = @('port=JTAG', 'mode=NORMAL', '-rst', '--reset', '-halt', '--halt', '--download', '--erase', '-go', '--run')
    for ($i = 0; $i -lt $lines.Count; ++$i) {
        $tokens = $lines[$i].Split([char]9)
        $offset = 4
        if ($Shared) {
            if ($tokens[$offset] -ne 'shared') { throw "Upload $i did not use shared connection." }
            $offset++
        } elseif ($tokens[$offset] -eq 'shared') {
            throw "Upload $i unexpectedly used shared connection."
        }
        if ($tokens.Count -ne ($offset + 5) -or
            $tokens[0] -ne '-c' -or $tokens[1] -ne 'port=SWD' -or
            $tokens[2] -ne 'mode=HOTPLUG' -or
            $tokens[3] -ne ("freq={0}" -f $ExpectedFrequencies[$i]) -or
            $tokens[$offset] -ne "sn=$serial" -or
            $tokens[$offset + 1] -ne '--upload' -or
            $tokens[$offset + 2] -ne $expectedAddresses[$i] -or
            [int]$tokens[$offset + 3] -ne $ExpectedSizes[$i]) {
            throw "Upload $i did not match HOTPLUG allowlist/address/size: $($lines[$i])"
        }
        foreach ($token in $tokens) {
            if ($forbidden -contains $token) { throw "Forbidden command argument '$token'." }
        }
    }
}

try {
    $success = Invoke-FakeExport -Name 'ok' -Mode 'success'
    if ($success.ExitCode -ne 0) { throw "Default fake export failed: $($success.Output)" }
    Assert-HotPlugUploadArguments -ArgsLog $success.ArgsLog -ExpectedFrequencies @('1000', '1000', '1000', '1000', '1000', '1000', '1000')
    $successRun = Get-ExportRunDirectory $success.CaseRoot
    $successCsv = Join-Path $successRun.FullName 'run.csv'
    $successMetaPath = Join-Path $successRun.FullName 'run-metadata.json'
    $successMetadata = Get-Content -LiteralPath $successMetaPath -Raw -Encoding UTF8 | ConvertFrom-Json
    if (-not (Test-Path -LiteralPath $successCsv -PathType Leaf) -or
        $successMetadata.status -ne 'success' -or $successMetadata.csv -ne $successCsv -or
        $successMetadata.requested_connection_frequency_khz -ne 1000 -or
        $successMetadata.connection_frequency_khz -ne 1000 -or
        $successMetadata.actual_connection_frequency_khz -ne 950 -or
        -not $successMetadata.shared_connection -or $successMetadata.probe_sn -ne $serial -or
        $successMetadata.flash_descriptor.status -ne 'matched' -or
        -not $successMetadata.status_snapshot) {
        throw 'Default shared export metadata, Flash descriptor, actual frequency, CSV, or nested status is missing.'
    }
    $statusJson = Get-Content -LiteralPath $successMetadata.status_snapshot -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($statusJson.flash_descriptor.status -ne 'matched' -or
        $statusJson.connection.probe_sn -ne $serial -or
        $statusJson.connection.actual_frequency_khz -ne 950) {
        throw 'Nested real PowerShell status process did not preserve its selected SN/rate/descriptor.'
    }

    $fallback = Invoke-FakeExport -Name 'fallback' `
        -Mode 'connect-error-1000' -Frequency '1000' -ActualFrequency '375'
    if ($fallback.ExitCode -ne 0) { throw "Connection-only fallback failed: $($fallback.Output)" }
    Assert-HotPlugUploadArguments -ArgsLog $fallback.ArgsLog `
        -ExpectedFrequencies @('1000', '400', '400', '400', '400', '400', '400', '400')
    $fallbackRun = Get-ExportRunDirectory $fallback.CaseRoot
    $fallbackMetadata = Get-Content -LiteralPath (Join-Path $fallbackRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($fallbackMetadata.connection_frequency_khz -ne 400 -or
        $fallbackMetadata.actual_connection_frequency_khz -ne 375 -or
        $fallbackMetadata.connection_attempts.Count -lt 5 -or
        $fallbackMetadata.connection_attempts[0].error_class -ne 'core_inaccessible') {
        throw 'Successful lower-rate fallback did not preserve initial failure, actual rate, and attempts.'
    }

    $postStatusFailure = Invoke-FakeExport -Name 'post-status' -Mode 'status-only-error'
    if ($postStatusFailure.ExitCode -ne 0) { throw "Valid CSV should survive status-reader failure: $($postStatusFailure.Output)" }
    $postStatusRun = Get-ExportRunDirectory $postStatusFailure.CaseRoot
    $postStatusMetadata = Get-Content -LiteralPath (Join-Path $postStatusRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($postStatusMetadata.status -ne 'success_with_warnings' -or
        -not (Test-Path -LiteralPath $postStatusMetadata.csv -PathType Leaf) -or
        [string]::IsNullOrWhiteSpace([string]$postStatusMetadata.diagnostic_failure) -or
        -not (Test-Path -LiteralPath $postStatusMetadata.status_snapshot -PathType Leaf)) {
        throw 'Post-export status failure erased the valid CSV or was not recorded as a warning.'
    }

    $wrongDescriptor = Join-Path $OutputRoot 'wrong-target-descriptor.bin'
    $wrongBytes = [IO.File]::ReadAllBytes($descriptorFixture)
    $wrongBytes[28] = $wrongBytes[28] -bxor 1
    [IO.File]::WriteAllBytes($wrongDescriptor, $wrongBytes)
    $mismatch = Invoke-FakeExport -Name 'wrong-elf' -Mode 'success' -DescriptorPath $wrongDescriptor
    if ($mismatch.ExitCode -eq 0) { throw 'Wrong ELF Flash descriptor unexpectedly passed.' }
    $mismatchRun = Get-ExportRunDirectory $mismatch.CaseRoot
    $mismatchMetadata = Get-Content -LiteralPath (Join-Path $mismatchRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $mismatchCalls = @(Get-Content -LiteralPath $mismatch.ArgsLog -Encoding UTF8)
    if ($mismatchMetadata.error_class -ne 'elf_mismatch' -or $mismatchCalls.Count -ne 1 -or
        (Test-Path -LiteralPath (Join-Path $mismatchRun.FullName 'header-initial.bin'))) {
        throw 'Flash descriptor mismatch was retried or RAM was read before ELF identity validation.'
    }

    $ambiguous = Invoke-FakeExport -Name 'ambiguous' `
        -Mode 'success' -PassProbeSN:$false -Probes '1234567890|0987654321'
    if ($ambiguous.ExitCode -eq 0) { throw 'Multiple probes without explicit ProbeSN were silently selected.' }
    $ambiguousRun = Get-ExportRunDirectory $ambiguous.CaseRoot
    $ambiguousMetadata = Get-Content -LiteralPath (Join-Path $ambiguousRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($ambiguousMetadata.error_class -ne 'probe_ambiguous' -or
        (Test-Path -LiteralPath $ambiguous.ArgsLog)) {
        throw 'Probe ambiguity did not fail before any target connection.'
    }

    $dryRoot = Join-Path $OutputRoot 'dry-run'
    $dryArgs = @('-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
        '-File', $exportScript, '-Elf', $resolvedElf, '-OutDirectory', $dryRoot,
        '-ProgrammerCLI', $fakeExe, '-ProbeSN', $serial, '-DryRun')
    $dryOutput = @(& $powershellExe @dryArgs)
    if ($LASTEXITCODE -ne 0) { throw 'Export DryRun failed.' }
    $dry = ($dryOutput -join [Environment]::NewLine) | ConvertFrom-Json
    if ($dry.requested_connection_frequency_khz -ne 1000 -or
        -not $dry.shared_connection -or $dry.probe_enumeration_arguments[1] -ne 'st-link-only' -or
        $dry.flash_descriptor_read[$dry.flash_descriptor_read.Count - 4] -ne '--upload') {
        throw 'DryRun did not show default 1000/shared, probe enumeration, or first Flash descriptor read.'
    }

    Write-Output 'PASS default 1000/shared, unique fixed probe, Flash-first ELF check, 1000-to-400 connection fallback, actual-rate capture, nested bool child process, CSV retention, probe ambiguity, and DryRun'
    Write-Output "Evidence: $OutputRoot"
}
finally {
    foreach ($name in $environmentNames) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name])
    }
}
