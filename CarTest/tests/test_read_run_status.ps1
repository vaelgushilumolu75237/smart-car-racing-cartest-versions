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
$reader = Join-Path $projectRoot 'tools\run_status_reader.py'
$fakeSource = Join-Path $PSScriptRoot 'support\FakeRunStatusCubeProgrammer.cs'
$python = (Get-Command -Name 'python.exe' -ErrorAction Stop).Source
$powershellExe = Join-Path $PSHOME 'powershell.exe'
$OutputRoot = [IO.Path]::GetFullPath($OutputRoot)
New-Item -ItemType Directory -Path $OutputRoot -Force | Out-Null

$fakeDirectory = Join-Path $OutputRoot 'fake cli with spaces'
New-Item -ItemType Directory -Path $fakeDirectory -Force | Out-Null
$fakeExe = Join-Path $fakeDirectory 'STM32_Programmer_CLI.exe'
Add-Type -TypeDefinition (Get-Content -LiteralPath $fakeSource -Raw -Encoding UTF8) `
    -OutputAssembly $fakeExe -OutputType ConsoleApplication

$resolvedElf = (Resolve-Path -LiteralPath $Elf).Path
$resolvedHeader = (Resolve-Path -LiteralPath $HeaderFixture).Path
$resolvedScalars = (Resolve-Path -LiteralPath $ScalarFixture).Path
$beforeHash = (Get-FileHash -LiteralPath $resolvedElf -Algorithm SHA256).Hash.ToLowerInvariant()
$planLines = @(& $python $reader plan --elf $resolvedElf)
if ($LASTEXITCODE -ne 0) { throw 'Could not inspect the selected ELF for the status fake CLI test.' }
$plan = ($planLines -join [Environment]::NewLine) | ConvertFrom-Json
if ($plan.scalar_read.size -gt 4096 -or $plan.header_read.size -ne 64) {
    throw 'The status reader planned a memory read outside its explicit bounds.'
}

$serial = '1234567890'
$savedEnvironment = @{}
$environmentNames = @(
    'RUN_STATUS_FAKE_ARGS_PATH', 'RUN_STATUS_FAKE_CALL_INDEX_PATH',
    'RUN_STATUS_FAKE_EXPECTED_SN', 'RUN_STATUS_FAKE_SCALAR_ADDRESS',
    'RUN_STATUS_FAKE_SCALAR_SIZE', 'RUN_STATUS_FAKE_HEADER_ADDRESS',
    'RUN_STATUS_FAKE_SCALAR_FIXTURE', 'RUN_STATUS_FAKE_HEADER_FIXTURE',
    'RUN_STATUS_FAKE_MODE'
)
foreach ($name in $environmentNames) { $savedEnvironment[$name] = [Environment]::GetEnvironmentVariable($name) }

function Assert-SafeUploadArguments {
    param([string]$ArgumentsPath)
    $lines = @(Get-Content -LiteralPath $ArgumentsPath -Encoding UTF8)
    if ($lines.Count -ne 2) { throw "Expected exactly 2 RAM status reads, found $($lines.Count)." }
    $forbidden = @('port=JTAG', 'mode=NORMAL', '-rst', '--reset', '-hardrst', '-halt', '--halt', '--download', '--erase', '-go', '--run')
    foreach ($line in $lines) {
        $tokens = $line.Split([char]9)
        if ($tokens.Count -ne 10 -or $tokens[0] -ne '-c' -or
            $tokens[1] -ne 'port=SWD' -or $tokens[2] -ne 'mode=HOTPLUG' -or
            $tokens[3] -ne 'freq=1000' -or $tokens[4] -ne 'shared' -or
            $tokens[5] -ne "sn=$serial" -or $tokens[6] -ne '--upload') {
            throw "Unexpected RAM-read argv: $line"
        }
        foreach ($token in $tokens) {
            if ($forbidden -contains $token) { throw "Forbidden operation argument observed: $token" }
        }
    }
    $one = $lines[0].Split([char]9)
    $two = $lines[1].Split([char]9)
    if ($one[7] -ne $plan.scalar_read.address -or
        [int]$one[8] -ne $plan.scalar_read.size -or
        $two[7] -ne $plan.header_read.address -or [int]$two[8] -ne 64) {
        throw 'The CLI read addresses/sizes do not match the actual ELF symbols and 64-byte header.'
    }
}

function Invoke-StatusCase {
    param([string]$Name, [string]$Mode, [switch]$DryRun, [string]$ElfPath = $resolvedElf)
    $caseRoot = Join-Path $OutputRoot $Name
    New-Item -ItemType Directory -Path $caseRoot -Force | Out-Null
    $argsLog = Join-Path $caseRoot 'fake-cli-args.tsv'
    $callIndex = Join-Path $caseRoot 'fake-cli-call-index.txt'
    $env:RUN_STATUS_FAKE_ARGS_PATH = $argsLog
    $env:RUN_STATUS_FAKE_CALL_INDEX_PATH = $callIndex
    $env:RUN_STATUS_FAKE_EXPECTED_SN = $serial
    $env:RUN_STATUS_FAKE_SCALAR_ADDRESS = $plan.scalar_read.address
    $env:RUN_STATUS_FAKE_SCALAR_SIZE = [string]$plan.scalar_read.size
    $env:RUN_STATUS_FAKE_HEADER_ADDRESS = $plan.header_read.address
    $env:RUN_STATUS_FAKE_SCALAR_FIXTURE = $resolvedScalars
    $env:RUN_STATUS_FAKE_HEADER_FIXTURE = $resolvedHeader
    $env:RUN_STATUS_FAKE_MODE = $Mode

    $outputRoot = Join-Path $caseRoot 'output root'
    $childArguments = @(
        '-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
        '-File', $wrapper,
        '-Elf', $ElfPath,
        '-OutDirectory', $outputRoot,
        '-ProgrammerCLI', $fakeExe,
        '-ProbeSN', $serial
    )
    if ($DryRun) { $childArguments += '-DryRun' }
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $childOutput = @(& $powershellExe @childArguments 2>&1)
        $exitCode = $LASTEXITCODE
    }
    finally { $ErrorActionPreference = $previousPreference }
    $text = (($childOutput | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine)
    Set-Content -LiteralPath (Join-Path $caseRoot 'wrapper-output.log') -Value $text -Encoding UTF8
    Set-Content -LiteralPath (Join-Path $caseRoot 'wrapper-exitcode.txt') -Value ([string]$exitCode) -Encoding ASCII
    return [PSCustomObject]@{ CaseRoot = $caseRoot; OutputRoot = $outputRoot; ArgsLog = $argsLog; ExitCode = $exitCode; Output = $text }
}

function Get-StatusRunDirectory {
    param([string]$Root)
    $dirs = @(Get-ChildItem -LiteralPath $Root -Directory -ErrorAction SilentlyContinue |
        Sort-Object LastWriteTime -Descending)
    if ($dirs.Count -eq 0) { return $null }
    return $dirs[0]
}

try {
    $success = Invoke-StatusCase -Name 'fake-cli-success-idle-header' -Mode 'success'
    if ($success.ExitCode -ne 0) { throw "Fake status read should succeed: $($success.Output)" }
    Assert-SafeUploadArguments -ArgumentsPath $success.ArgsLog
    $successRun = Get-StatusRunDirectory -Root $success.OutputRoot
    if (-not $successRun) { throw 'Successful fake status read did not create a timestamped diagnostics directory.' }
    $statusPath = Join-Path $successRun.FullName 'run-status.json'
    $status = Get-Content -LiteralPath $statusPath -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($status.status -ne 'success' -or $status.runtime_log_header.state_name -ne 'idle' -or
        $status.runtime_log_header.count -ne 0 -or
        -not $status.runtime_log_header.build_id_matches_elf -or
        $status.runtime_log_header.header_crc_status -ne 'valid' -or
        $status.runtime_log_header.record_payload_read -or $status.diagnostic_is_run_record) {
        throw 'The current matching ELF/header did not produce an IDLE-only diagnostic status.'
    }
    foreach ($required in @('scalar-region.bin', 'runtime-log-header.bin', 'read-plan.json',
                            'python-plan.log', 'python-decode.log',
                            'cubeprogrammer-scalars.log', 'cubeprogrammer-header.log')) {
        if (-not (Test-Path -LiteralPath (Join-Path $successRun.FullName $required) -PathType Leaf)) {
            throw "Successful diagnostic did not preserve $required."
        }
    }

    $failure = Invoke-StatusCase -Name 'fake-cli-connect-error' -Mode 'connect-error'
    if ($failure.ExitCode -eq 0) { throw 'Fake DEV_CONNECT_ERR was incorrectly reported as success.' }
    $failureRun = Get-StatusRunDirectory -Root $failure.OutputRoot
    if (-not $failureRun) { throw 'Failed fake CLI read did not retain its diagnostics directory.' }
    $failureStatus = Get-Content -LiteralPath (Join-Path $failureRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $failureCalls = @(Get-Content -LiteralPath $failure.ArgsLog -Encoding UTF8)
    if ($failureStatus.status -ne 'failed' -or $failureCalls.Count -ne 1 -or
        $failureStatus.error -notmatch 'DEV_CONNECT_ERR' -or
        (Test-Path -LiteralPath (Join-Path $failureRun.FullName 'runtime-log-header.bin'))) {
        throw 'Connection failure must remain failed and must not invent a header/scalar snapshot.'
    }

    $dry = Invoke-StatusCase -Name 'dry-run-no-cli' -Mode 'success' -DryRun
    if ($dry.ExitCode -ne 0) { throw "DryRun should succeed without connecting: $($dry.Output)" }
    $dryJson = $dry.Output | ConvertFrom-Json
    if ($dryJson.status -ne 'dry_run' -or $dryJson.connected -or
        $dryJson.reads.Count -ne 2 -or (Test-Path -LiteralPath $dry.ArgsLog) -or
        (Test-Path -LiteralPath $dry.OutputRoot)) {
        throw 'DryRun must print the two plans and leave the fake CLI/output directory untouched.'
    }
    foreach ($read in $dryJson.reads) {
        $tokens = @($read.arguments)
        if ($tokens[0] -ne '-c' -or $tokens[1] -ne 'port=SWD' -or
            $tokens[2] -ne 'mode=HOTPLUG' -or $tokens[3] -ne 'freq=1000' -or
            $tokens[4] -ne 'shared' -or $tokens[6] -ne '--upload') {
            throw 'DryRun printed an unsafe or incomplete CubeProgrammer command.'
        }
        if (@($tokens | Where-Object { $_ -in @('-rst', '--reset', '-halt', '--halt', '--download', '--erase', '-go', '--run') }).Count -gt 0) {
            throw 'DryRun included a forbidden reset, halt, flash, or run option.'
        }
    }

    $missingPath = Join-Path (Join-Path $OutputRoot 'missing-elf-preflight-failure') 'does-not-exist.elf'
    $missing = Invoke-StatusCase -Name 'missing-elf-preflight-failure' -Mode 'success' -ElfPath $missingPath
    if ($missing.ExitCode -eq 0) { throw 'Missing ELF unexpectedly succeeded.' }
    $missingRun = Get-StatusRunDirectory -Root $missing.OutputRoot
    if (-not $missingRun) { throw 'Missing ELF did not produce preflight failure JSON.' }
    $missingStatus = Get-Content -LiteralPath (Join-Path $missingRun.FullName 'run-status.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($missingStatus.status -ne 'failed' -or $missingStatus.stage -ne 'preflight' -or
        [string]::IsNullOrWhiteSpace([string]$missingStatus.script_stack_trace)) {
        throw 'Missing ELF failure must have status=failed and a preserved stack location.'
    }

    $afterHash = (Get-FileHash -LiteralPath $resolvedElf -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($afterHash -ne $beforeHash) { throw 'Status diagnostic modified the selected ELF.' }
    Write-Output 'PASS fake CLI success/IDLE header, two bounded HOTPLUG reads, DEV_CONNECT_ERR failure, safe DryRun, missing-ELF failure, and unchanged ELF'
}
finally {
    foreach ($name in $environmentNames) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name])
    }
}
