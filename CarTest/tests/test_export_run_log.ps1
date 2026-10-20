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

function Set-U16LE {
    param([byte[]]$Bytes, [int]$Offset, [uint16]$Value)
    [Array]::Copy([BitConverter]::GetBytes($Value), 0, $Bytes, $Offset, 2)
}

function Set-U32LE {
    param([byte[]]$Bytes, [int]$Offset, [uint32]$Value)
    [Array]::Copy([BitConverter]::GetBytes($Value), 0, $Bytes, $Offset, 4)
}

function Get-Crc32 {
    param([byte[]]$Bytes)
    [uint32]$crc = [uint32]::MaxValue
    [uint32]$polynomial = 3988292384
    foreach ($value in $Bytes) {
        $crc = $crc -bxor [uint32]$value
        for ($bit = 0; $bit -lt 8; ++$bit) {
            if (($crc -band 1) -ne 0) {
                $crc = ($crc -shr 1) -bxor $polynomial
            } else {
                $crc = $crc -shr 1
            }
        }
    }
    return [uint32]($crc -bxor [uint32]::MaxValue)
}

function Update-RuntimeHeaderCrc {
    param([byte[]]$Bytes)
    [Array]::Clear($Bytes, 40, 4)
    Set-U32LE -Bytes $Bytes -Offset 40 -Value (Get-Crc32 -Bytes $Bytes)
}

function Write-RuntimeDiagnosticFixture {
    param(
        [string]$Path,
        [uint32]$State,
        [uint32]$Count,
        [uint32]$SessionId,
        [uint32]$StartEvents,
        [uint32]$StopEvents,
        [uint32]$StartToken
    )
    $bytes = New-Object byte[] 64
    Set-U32LE $bytes 0 0x47414944
    Set-U16LE $bytes 4 1
    Set-U16LE $bytes 6 64
    Set-U32LE $bytes 8 2
    Set-U32LE $bytes 16 $StartEvents
    Set-U32LE $bytes 20 $StopEvents
    Set-U32LE $bytes 24 8
    Set-U32LE $bytes 28 $StartToken
    Set-U32LE $bytes 32 $State
    Set-U32LE $bytes 36 $Count
    Set-U32LE $bytes 40 ([uint32]$fixtureHeader.stop_ms)
    Set-U16LE $bytes 54 1
    Set-U32LE $bytes 56 $SessionId
    Set-U32LE $bytes 12 (Get-Crc32 -Bytes $bytes)
    [IO.File]::WriteAllBytes($Path, $bytes)
}

function Write-SpeedPiDiagnosticFixture {
    param([string]$Path, [uint32]$StartToken, [uint32]$Flags, [uint32]$PiType)
    $bytes = New-Object byte[] 128
    Set-U32LE $bytes 0 0x47414950
    Set-U16LE $bytes 4 1
    Set-U16LE $bytes 6 128
    Set-U32LE $bytes 8 2
    Set-U32LE $bytes 16 $StartToken
    Set-U32LE $bytes 20 1000
    Set-U32LE $bytes 24 ([uint32]$fixtureHeader.stop_ms)
    Set-U32LE $bytes 28 ([uint32]$fixtureHeader.stop_ms - 10)
    Set-U32LE $bytes 32 2400
    Set-U32LE $bytes 36 2300
    Set-U32LE $bytes 40 2200
    Set-U32LE $bytes 44 2100
    Set-U16LE $bytes 48 500
    Set-U16LE $bytes 50 490
    Set-U16LE $bytes 52 700
    Set-U16LE $bytes 54 690
    Set-U16LE $bytes 56 680
    Set-U16LE $bytes 58 670
    Set-U32LE $bytes 60 3
    Set-U32LE $bytes 64 0
    Set-U32LE $bytes 68 100
    Set-U32LE $bytes 72 50
    Set-U32LE $bytes 76 $Flags
    Set-U32LE $bytes 80 $PiType
    Set-U32LE $bytes 12 (Get-Crc32 -Bytes $bytes)
    [IO.File]::WriteAllBytes($Path, $bytes)
}

$idleHeaderPath = Join-Path $OutputRoot 'target-runtime-log-header-idle.bin'
$idleHeader = New-Object byte[] 64
[Array]::Copy($fixtureHeaderBytes, $idleHeader, 64)
$idleHeader[14] = 0
$idleHeader[15] = 0
[Array]::Clear($idleHeader, 16, 28)
$idleHeader[60] = 0
[Array]::Clear($idleHeader, 61, 3)
Update-RuntimeHeaderCrc $idleHeader
[IO.File]::WriteAllBytes($idleHeaderPath, $idleHeader)

$activeHeaderPath = Join-Path $OutputRoot 'target-runtime-log-header-active.bin'
$activeHeader = New-Object byte[] 64
[Array]::Copy($fixtureHeaderBytes, $activeHeader, 64)
$activeHeader[14] = 1
[Array]::Clear($activeHeader, 28, 4)
$activeHeader[60] = 0
Update-RuntimeHeaderCrc $activeHeader
[IO.File]::WriteAllBytes($activeHeaderPath, $activeHeader)

$frozenDiagnosticPath = Join-Path $OutputRoot 'target-runtime-diagnostic-frozen.bin'
Write-RuntimeDiagnosticFixture -Path $frozenDiagnosticPath -State ([uint32]$fixtureHeader.state) `
    -Count ([uint32]$fixtureHeader.count) -SessionId ([uint32]$fixtureHeader.session_id) `
    -StartEvents 1 -StopEvents 1 -StartToken 1
$idleDiagnosticPath = Join-Path $OutputRoot 'target-runtime-diagnostic-idle.bin'
Write-RuntimeDiagnosticFixture -Path $idleDiagnosticPath -State 0 -Count 0 -SessionId 0 `
    -StartEvents 0 -StopEvents 0 -StartToken 0
$activeDiagnosticPath = Join-Path $OutputRoot 'target-runtime-diagnostic-active.bin'
Write-RuntimeDiagnosticFixture -Path $activeDiagnosticPath -State 1 `
    -Count ([uint32]$fixtureHeader.count) -SessionId ([uint32]$fixtureHeader.session_id) `
    -StartEvents 1 -StopEvents 0 -StartToken 1
$speedPiDiagnosticIdlePath = Join-Path $OutputRoot 'target-speed-pi-diagnostic-idle.bin'
$speedPiDiagnosticActivePath = Join-Path $OutputRoot 'target-speed-pi-diagnostic-active.bin'
$speedPiDiagnosticFrozenPath = Join-Path $OutputRoot 'target-speed-pi-diagnostic-frozen.bin'
if ($statusPlan.speed_pi_diagnostic_snapshot.available) {
    Write-SpeedPiDiagnosticFixture -Path $speedPiDiagnosticIdlePath -StartToken 0 -Flags 0 -PiType 0
    Write-SpeedPiDiagnosticFixture -Path $speedPiDiagnosticActivePath -StartToken 1 `
        -Flags 15 -PiType 2
    Write-SpeedPiDiagnosticFixture -Path $speedPiDiagnosticFrozenPath -StartToken 1 `
        -Flags 27 -PiType 2
}

$environmentNames = @(
    'RUNTIME_LOG_FAKE_BUFFER_PATH', 'RUNTIME_LOG_FAKE_ARGS_PATH',
    'RUNTIME_LOG_FAKE_CALL_INDEX_PATH', 'RUNTIME_LOG_FAKE_EXPECTED_ADDRESS',
    'RUNTIME_LOG_FAKE_EXPECTED_SN', 'RUNTIME_LOG_FAKE_FREQ',
    'RUNTIME_LOG_FAKE_SHARED', 'RUNTIME_LOG_FAKE_MODE',
    'RUNTIME_LOG_FAKE_DESCRIPTOR_ADDRESS', 'RUNTIME_LOG_FAKE_DESCRIPTOR_PATH',
    'RUNTIME_LOG_FAKE_DIAGNOSTIC_ADDRESS', 'RUNTIME_LOG_FAKE_DIAGNOSTIC_PATH',
    'RUNTIME_LOG_FAKE_SPEED_PI_DIAGNOSTIC_ADDRESS', 'RUNTIME_LOG_FAKE_SPEED_PI_DIAGNOSTIC_PATH',
    'RUNTIME_LOG_FAKE_HEADER_OVERRIDE_PATH',
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
    $env:RUNTIME_LOG_FAKE_DIAGNOSTIC_ADDRESS = [string]$statusPlan.runtime_diagnostic_snapshot.address
    $env:RUNTIME_LOG_FAKE_DIAGNOSTIC_PATH = switch ($Mode) {
        'idle' { $idleDiagnosticPath }
        'active' { $activeDiagnosticPath }
        default { $frozenDiagnosticPath }
    }
    $env:RUNTIME_LOG_FAKE_SPEED_PI_DIAGNOSTIC_ADDRESS = [string]$statusPlan.speed_pi_diagnostic_snapshot.address
    $env:RUNTIME_LOG_FAKE_SPEED_PI_DIAGNOSTIC_PATH = switch ($Mode) {
        'idle' { $speedPiDiagnosticIdlePath }
        'active' { $speedPiDiagnosticActivePath }
        default { $speedPiDiagnosticFrozenPath }
    }
    $env:RUNTIME_LOG_FAKE_HEADER_OVERRIDE_PATH = switch ($Mode) {
        'idle' { $idleHeaderPath }
        'active' { $activeHeaderPath }
        default { '' }
    }
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

$successAddresses = @(
    [string]$layout.descriptor_address,
    [string]$layout.descriptor_address,
    [string]$statusPlan.scalar_read.address,
    [string]$statusPlan.header_read.address,
    [string]$statusPlan.runtime_diagnostic_snapshot.address,
    [string]$statusPlan.header_read.address,
    [string]$layout.storage_address,
    [string]$layout.storage_address,
    [string]$layout.storage_address,
    [string]$layout.descriptor_address,
    [string]$statusPlan.scalar_read.address,
    [string]$statusPlan.header_read.address,
    [string]$statusPlan.runtime_diagnostic_snapshot.address,
    [string]$statusPlan.header_read.address
)
$successSizes = @(
    64, 64, [int]$statusPlan.scalar_read.size, 64, 64, 64,
    64, [int]$fixtureHeader.payload_size, 64,
    64, [int]$statusPlan.scalar_read.size, 64, 64, 64
)
if ($statusPlan.speed_pi_diagnostic_snapshot.available) {
    $successAddresses = @($successAddresses[0..4]) +
        @([string]$statusPlan.speed_pi_diagnostic_snapshot.address) +
        @($successAddresses[5..12]) +
        @([string]$statusPlan.speed_pi_diagnostic_snapshot.address) +
        @($successAddresses[13])
    $successSizes = @($successSizes[0..4]) + @(128) +
        @($successSizes[5..12]) + @(128) + @($successSizes[13])
}

function Assert-HotPlugUploadArguments {
    param(
        [string]$ArgsLog,
        [string[]]$ExpectedFrequencies,
        [bool]$Shared = $true,
        [int[]]$ExpectedSizes,
        [string[]]$ExpectedAddresses
    )
    $lines = @(Get-Content -LiteralPath $ArgsLog -Encoding UTF8)
    if ($null -eq $ExpectedSizes) { $ExpectedSizes = @($successSizes | Select-Object -First $ExpectedFrequencies.Count) }
    if ($null -eq $ExpectedAddresses) { $ExpectedAddresses = @($successAddresses | Select-Object -First $ExpectedFrequencies.Count) }
    if ($lines.Count -ne $ExpectedFrequencies.Count -or $lines.Count -ne $ExpectedSizes.Count) {
        throw "Expected $($ExpectedFrequencies.Count) uploads, found $($lines.Count)."
    }
    if ($ExpectedAddresses.Count -ne $ExpectedFrequencies.Count) {
        throw "Expected address count $($ExpectedAddresses.Count) does not match upload count $($ExpectedFrequencies.Count)."
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
            $tokens[$offset + 2] -ne $ExpectedAddresses[$i] -or
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
    $successFrequencies = @(1..$successAddresses.Count | ForEach-Object { '1000' })
    Assert-HotPlugUploadArguments -ArgsLog $success.ArgsLog -ExpectedFrequencies $successFrequencies
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
        $successMetadata.export_diagnostic_status -ne 'verified' -or
        $successMetadata.status -ne 'success' -or
        -not $successMetadata.status_post_export -or
        -not $successMetadata.status_snapshot) {
        throw 'Default export did not produce a validated CSV with stable post-export diagnostics and matching Flash descriptor.'
    }
    if ($statusPlan.speed_pi_diagnostic_snapshot.available -and
        (-not (Test-Path -LiteralPath $successMetadata.speed_pi_diagnostic_json -PathType Leaf) -or
         $successMetadata.speed_pi_diagnostic_preflight.start_token -ne 1 -or
         -not $successMetadata.speed_pi_diagnostic_preflight.frozen -or
         -not $successMetadata.speed_pi_diagnostic_post_export.frozen)) {
        throw 'Speed PI diagnostic was not exported beside the CSV with a matching frozen PE7 token.'
    }
    $statusJson = Get-Content -LiteralPath $successMetadata.status_snapshot -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($statusJson.flash_descriptor.status -ne 'matched' -or
        $statusJson.connection.probe_sn -ne $serial -or
        $statusJson.connection.actual_frequency_khz -ne 950 -or
        $statusJson.status_class -ne 'frozen' -or
        -not $statusJson.runtime_diagnostic_snapshot.crc_valid -or
        $statusJson.runtime_diagnostic_snapshot.cpu_header_state_name -ne 'frozen') {
        throw 'Nested real PowerShell status process did not preserve its selected SN/rate/descriptor.'
    }

    $fallback = Invoke-FakeExport -Name 'fallback' `
        -Mode 'connect-error-1000' -Frequency '1000' -ActualFrequency '375'
    if ($fallback.ExitCode -ne 0) { throw "Connection-only fallback failed: $($fallback.Output)" }
    $fallbackFrequencies = @('1000') + @(1..$successAddresses.Count | ForEach-Object { '400' })
    $fallbackAddresses = @([string]$layout.descriptor_address) + $successAddresses
    $fallbackSizes = @(64) + $successSizes
    Assert-HotPlugUploadArguments -ArgsLog $fallback.ArgsLog `
        -ExpectedFrequencies $fallbackFrequencies -ExpectedAddresses $fallbackAddresses `
        -ExpectedSizes $fallbackSizes
    $fallbackRun = Get-ExportRunDirectory $fallback.CaseRoot
    $fallbackMetadata = Get-Content -LiteralPath (Join-Path $fallbackRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($fallbackMetadata.connection_frequency_khz -ne 400 -or
        $fallbackMetadata.actual_connection_frequency_khz -ne 375 -or
        $fallbackMetadata.connection_attempts.Count -lt 5 -or
        $fallbackMetadata.connection_attempts[0].error_class -ne 'core_inaccessible') {
        throw 'Successful lower-rate fallback did not preserve initial failure, actual rate, and attempts.'
    }

    foreach ($stateCase in @(
        @{ Name = 'idle'; ExpectedClass = 'neverstarted' },
        @{ Name = 'active'; ExpectedClass = 'active' }
    )) {
        $stateResult = Invoke-FakeExport -Name $stateCase.Name -Mode $stateCase.Name
        if ($stateResult.ExitCode -ne 2) {
            throw "Stable $($stateCase.Name) status returned $($stateResult.ExitCode), expected no-run exit 2: $($stateResult.Output)"
        }
        $stateReadCount = 6 + [int][bool]$statusPlan.speed_pi_diagnostic_snapshot.available
        $stateFrequencies = @(1..$stateReadCount | ForEach-Object { '1000' })
        Assert-HotPlugUploadArguments -ArgsLog $stateResult.ArgsLog `
            -ExpectedFrequencies $stateFrequencies
        $stateRun = Get-ExportRunDirectory $stateResult.CaseRoot
        $stateMetadata = Get-Content -LiteralPath (Join-Path $stateRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
        $stateSnapshot = Get-Content -LiteralPath (Join-Path $stateRun.FullName 'status-preflight.json') -Raw -Encoding UTF8 | ConvertFrom-Json
        if ($stateMetadata.run_state_class -ne $stateCase.ExpectedClass -or
            $stateSnapshot.status_class -ne $stateCase.ExpectedClass -or
            (Test-Path -LiteralPath (Join-Path $stateRun.FullName 'run.csv')) -or
            $stateMetadata.csv) {
            throw "Stable $($stateCase.Name) status was reported as a successful CSV export."
        }
    }

    if ($statusPlan.speed_pi_diagnostic_snapshot.available) {
        foreach ($piFailureCase in @(
            @{ Name = 'pi-crc-error'; Mode = 'speed-pi-crc-error' },
            @{ Name = 'pi-token-mismatch'; Mode = 'speed-pi-token-mismatch' }
        )) {
            $piFailure = Invoke-FakeExport -Name $piFailureCase.Name -Mode $piFailureCase.Mode
            if ($piFailure.ExitCode -eq 0) {
                throw "Speed PI $($piFailureCase.Name) unexpectedly reported a successful export."
            }
            $piFailureRun = Get-ExportRunDirectory $piFailure.CaseRoot
            $piFailureMetadata = Get-Content -LiteralPath (Join-Path $piFailureRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
            if ((Test-Path -LiteralPath (Join-Path $piFailureRun.FullName 'run.csv')) -or
                $piFailureMetadata.status -eq 'success' -or $piFailureMetadata.csv) {
                throw "Speed PI $($piFailureCase.Name) failure was reported as a successful CSV export."
            }
        }
    }

    $postDiagnosticFailure = Invoke-FakeExport -Name 'post-diagnostic-error' -Mode 'post-diagnostic-error'
    if ($postDiagnosticFailure.ExitCode -ne 3) {
        throw "Post-export diagnostic failure should return exit 3 while retaining the decoded CSV: $($postDiagnosticFailure.Output)"
    }
    $postDiagnosticRun = Get-ExportRunDirectory $postDiagnosticFailure.CaseRoot
    $postDiagnosticMetadata = Get-Content -LiteralPath (Join-Path $postDiagnosticRun.FullName 'run-metadata.json') -Raw -Encoding UTF8 | ConvertFrom-Json
    $successPrefixEnd = if ($statusPlan.speed_pi_diagnostic_snapshot.available) { 9 } else { 8 }
    $postFailureAddresses = @($successAddresses[0..$successPrefixEnd]) + @(
        [string]$layout.descriptor_address,
        [string]$statusPlan.scalar_read.address,
        [string]$statusPlan.header_read.address,
        [string]$statusPlan.runtime_diagnostic_snapshot.address,
        [string]$statusPlan.header_read.address,
        [string]$statusPlan.header_read.address,
        [string]$statusPlan.runtime_diagnostic_snapshot.address,
        [string]$statusPlan.header_read.address,
        [string]$statusPlan.header_read.address,
        [string]$statusPlan.runtime_diagnostic_snapshot.address,
        [string]$statusPlan.header_read.address
    )
    $postFailureSizes = @($successSizes[0..$successPrefixEnd]) + @(
        64, [int]$statusPlan.scalar_read.size, 64, 64, 64,
        64, 64, 64, 64, 64, 64
    )
    if ($statusPlan.speed_pi_diagnostic_snapshot.available) {
        $firstPiReadIndex = $successPrefixEnd + 4
        $secondPiReadIndex = $successPrefixEnd + 7
        $thirdPiReadIndex = $successPrefixEnd + 10
        $postFailureAddresses = @($postFailureAddresses[0..$firstPiReadIndex]) +
            @([string]$statusPlan.speed_pi_diagnostic_snapshot.address) +
            @($postFailureAddresses[($firstPiReadIndex + 1)..$secondPiReadIndex]) +
            @([string]$statusPlan.speed_pi_diagnostic_snapshot.address) +
            @($postFailureAddresses[($secondPiReadIndex + 1)..$thirdPiReadIndex]) +
            @([string]$statusPlan.speed_pi_diagnostic_snapshot.address) +
            @($postFailureAddresses[($thirdPiReadIndex + 1)..($postFailureAddresses.Count - 1)])
        $postFailureSizes = @($postFailureSizes[0..$firstPiReadIndex]) + @(128) +
            @($postFailureSizes[($firstPiReadIndex + 1)..$secondPiReadIndex]) + @(128) +
            @($postFailureSizes[($secondPiReadIndex + 1)..$thirdPiReadIndex]) + @(128) +
            @($postFailureSizes[($thirdPiReadIndex + 1)..($postFailureSizes.Count - 1)])
    }
    $postFailureFrequencies = @(1..$postFailureAddresses.Count | ForEach-Object { '1000' })
    Assert-HotPlugUploadArguments -ArgsLog $postDiagnosticFailure.ArgsLog `
        -ExpectedFrequencies $postFailureFrequencies -ExpectedAddresses $postFailureAddresses `
        -ExpectedSizes $postFailureSizes
    if ($postDiagnosticMetadata.status -ne 'decoded_csv_diagnostic_unverified' -or
        $postDiagnosticMetadata.export_diagnostic_status -ne 'unverified' -or
        -not (Test-Path -LiteralPath $postDiagnosticMetadata.csv -PathType Leaf) -or
        [string]::IsNullOrWhiteSpace([string]$postDiagnosticMetadata.diagnostic_failure)) {
        throw 'Post-export diagnostic failure did not return nonzero, retain run.csv, and record an unverified diagnostic status.'
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

    Write-Output ("PASS {0}-read HOTPLUG export sequence, CRC/token-validated frozen speed PI sidecar, CRC-validated runtime diagnostic fixture, IDLE/ACTIVE and corrupt-PI no-CSV failures, postflight failure exit 3 with CSV retention, fallback, Flash-first identity, probe ambiguity, and DryRun" -f $successAddresses.Count)
    Write-Output "Evidence: $OutputRoot"
}
finally {
    foreach ($name in $environmentNames) {
        [Environment]::SetEnvironmentVariable($name, $savedEnvironment[$name])
    }
}
