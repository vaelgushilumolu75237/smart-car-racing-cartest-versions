[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Elf,

    [string]$OutDirectory,

    [string]$ProgrammerCLI,

    [ValidatePattern('^[A-Za-z0-9_-]{1,64}$')]
    [string]$ProbeSN,

    [ValidateSet(100, 400, 1000)]
    [int]$ConnectionFrequencyKhz = 1000,

    [switch]$SharedConnection,

    [string]$Python = 'python.exe',

    [switch]$DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:ProjectDirectory = Split-Path -Parent $PSScriptRoot
$script:PythonExe = $null
$script:ProgrammerPath = $null
$script:RunDirectory = $null
$script:Stage = 'preflight'
$script:RunCsvPath = $null
$script:DecodeMetadataPath = $null
$script:StatusSnapshotPath = $null
$script:StatusPreflightPath = $null
$script:StatusPostExportPath = $null
$script:SpeedPiDiagnosticPreflight = $null
$script:SpeedPiDiagnosticPostExport = $null
$script:SpeedPiDiagnosticPreflightRaw = $null
$script:SpeedPiDiagnosticPostExportRaw = $null
$script:SpeedPiDiagnosticJsonPath = $null
$script:SpeedPiTracePreflight = $null
$script:SpeedPiTracePostExport = $null
$script:SpeedPiTracePreflightHeaderRaw = $null
$script:SpeedPiTracePostExportHeaderRaw = $null
$script:SpeedPiTraceRawPath = $null
$script:SpeedPiTraceCsvPath = $null
$script:SpeedPiTraceMetadataPath = $null
$script:SpeedPiTraceExportStatus = 'not_available'
$script:ReceiveDiagnosticPreflightRaw = $null
$script:RunStateClass = $null
$script:ExportDiagnosticStatus = 'not_started'
$script:DiagnosticFailure = $null
$script:CliLogs = New-Object System.Collections.Generic.List[string]
$script:StartedAtUtc = [DateTime]::UtcNow.ToString('o')
$script:SharedConnectionValue = $true
if ($PSBoundParameters.ContainsKey('SharedConnection')) {
    $script:SharedConnectionValue = [bool]$SharedConnection.IsPresent
}
$script:StartingFrequencyKhz = $ConnectionFrequencyKhz
$script:ActiveFrequencyKhz = $ConnectionFrequencyKhz
$script:ActualFrequencyKhz = $null
$script:SelectedProbeSN = $null
$script:ConnectionAttempts = New-Object System.Collections.Generic.List[object]
$script:ErrorClass = $null
$script:Connected = $false
$script:RunCsvValidated = $false
$script:FlashDescriptorValidation = $null

function Resolve-ProgrammerCli {
    if ($ProgrammerCLI) {
        $candidate = (Resolve-Path -LiteralPath $ProgrammerCLI -ErrorAction Stop).Path
        if ((Split-Path -Leaf $candidate) -ne 'STM32_Programmer_CLI.exe') {
            throw 'ProgrammerCLI must point to STM32_Programmer_CLI.exe.'
        }
        return $candidate
    }

    $bundleRoot = $null
    if ($env:LOCALAPPDATA) {
        $bundleRoot = Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer'
    }
    if ($bundleRoot -and (Test-Path -LiteralPath $bundleRoot -PathType Container)) {
        $versions = @(Get-ChildItem -LiteralPath $bundleRoot -Directory -ErrorAction SilentlyContinue)
        $versions = @($versions | ForEach-Object {
            $parsedVersion = $null
            if ([Version]::TryParse($_.Name, [ref]$parsedVersion)) {
                [PSCustomObject]@{ Path = $_.FullName; Version = $parsedVersion }
            }
        } | Sort-Object -Property Version -Descending)
        foreach ($version in $versions) {
            $bundledCli = Join-Path $version.Path 'bin\STM32_Programmer_CLI.exe'
            if (Test-Path -LiteralPath $bundledCli -PathType Leaf) {
                return (Resolve-Path -LiteralPath $bundledCli).Path
            }
        }
    }

    $standaloneCandidates = @()
    if ($env:ProgramFiles) {
        $standaloneCandidates += Join-Path $env:ProgramFiles 'STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    }
    if (${env:ProgramFiles(x86)}) {
        $standaloneCandidates += Join-Path ${env:ProgramFiles(x86)} 'STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    }
    if ($env:LOCALAPPDATA) {
        $standaloneCandidates += Join-Path $env:LOCALAPPDATA 'Programs\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    }
    foreach ($candidate in $standaloneCandidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }

    $fromPath = Get-Command -Name 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue
    if ($fromPath -and (Test-Path -LiteralPath $fromPath.Source)) {
        return $fromPath.Source
    }

    throw 'STM32_Programmer_CLI.exe was not found. Install STM32CubeProgrammer or pass -ProgrammerCLI with its full path.'
}

function Resolve-PythonExe {
    $command = Get-Command -Name $Python -ErrorAction Stop
    if ($command.CommandType -ne 'Application') {
        throw 'Python must resolve to a python.exe application.'
    }
    return $command.Source
}

function Resolve-ProbeSerial {
    $logPath = Join-Path $script:RunDirectory 'probe-list.log'
    $arguments = @('-l', 'st-link-only')
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = @(& $script:ProgrammerPath @arguments 2>&1)
        $exitCode = $LASTEXITCODE
    } finally { $ErrorActionPreference = $previousPreference }
    $text = (($output | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine)
    Set-Content -LiteralPath $logPath -Value $text -Encoding UTF8
    if ($exitCode -ne 0) {
        $script:ErrorClass = 'probe_absent'
        throw "ST-Link probe enumeration failed (exit $exitCode). See $logPath. $text"
    }
    $serials = @([regex]::Matches($text, '(?im)^\s*ST-LINK\s+SN\s*:\s*([A-Za-z0-9_-]+)\s*$') |
        ForEach-Object { $_.Groups[1].Value } | Sort-Object -Unique)
    if ($ProbeSN) {
        $match = @($serials | Where-Object { $_ -ieq $ProbeSN })
        if ($match.Count -ne 1) {
            $script:ErrorClass = 'probe_absent'
            throw "Requested ST-Link probe '$ProbeSN' was not found by '-l st-link-only'. See $logPath."
        }
        $script:SelectedProbeSN = $match[0]
    } elseif ($serials.Count -eq 1) {
        $script:SelectedProbeSN = $serials[0]
    } elseif ($serials.Count -eq 0) {
        $script:ErrorClass = 'probe_absent'
        throw "No ST-Link probe was found by '-l st-link-only'. See $logPath."
    } else {
        $script:ErrorClass = 'probe_ambiguous'
        throw "Multiple ST-Link probes were found; pass -ProbeSN explicitly. Found: $($serials -join ', '). See $logPath."
    }
    $selection = [ordered]@{
        enumeration_arguments = $arguments
        selected_probe_sn = $script:SelectedProbeSN
        enumerated_probe_serials = $serials
        selection_was_explicit = [bool]$ProbeSN
    }
    $selection | ConvertTo-Json -Depth 5 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'probe-selection.json') -Encoding UTF8
}

function Get-ConnectionFrequencies {
    if ($script:Connected) { return @($script:ActiveFrequencyKhz) }
    return @(@(1000, 400, 100) | Where-Object { $_ -le $script:StartingFrequencyKhz })
}

function Resolve-ToolErrorClass {
    param([string]$Message, [string]$Stage)
    if ($script:ErrorClass) { return $script:ErrorClass }
    if ($Stage -match 'descriptor' -and $Message -match 'descriptor mismatch') { return 'elf_mismatch' }
    if ($Message -match 'DEV_CONNECT_ERR|Unable to get core ID|No STM32 target found|Cannot connect to target|cannot connect to target') { return 'core_inaccessible' }
    if ($Stage -match 'decode|validate|python|header' -or $Message -match 'CRC|schema|header|descriptor') { return 'decode' }
    return 'ram_read'
}

function Save-ConnectionAttempts {
    if (-not $script:RunDirectory) { return }
    $script:ConnectionAttempts.ToArray() | ConvertTo-Json -Depth 8 |
        Set-Content -LiteralPath (Join-Path $script:RunDirectory 'connection-attempts.json') -Encoding UTF8
}

function Invoke-PythonJson {
    param(
        [Parameter(Mandatory = $true)][string[]]$Arguments,
        [Parameter(Mandatory = $true)][string]$LogPath
    )
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = @(& $script:PythonExe @Arguments 2>&1)
        $exitCode = $LASTEXITCODE
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }
    $stdoutLines = New-Object System.Collections.Generic.List[string]
    $logLines = New-Object System.Collections.Generic.List[string]
    foreach ($line in $output) {
        if ($line -is [System.Management.Automation.ErrorRecord]) {
            $logLines.Add("stderr: {0}" -f $line.ToString())
        }
        else {
            $stdoutLines.Add($line.ToString())
            $logLines.Add("stdout: {0}" -f $line.ToString())
        }
    }
    $text = $logLines -join [Environment]::NewLine
    Set-Content -LiteralPath $LogPath -Value $text -Encoding UTF8
    if ($exitCode -ne 0) {
        throw "Offline ELF/log validator failed (exit $exitCode). See $LogPath. $text"
    }
    try {
        return (($stdoutLines -join [Environment]::NewLine) | ConvertFrom-Json -ErrorAction Stop)
    }
    catch {
        throw "Offline ELF/log validator returned invalid JSON. See $LogPath."
    }
}

function Get-HotPlugConnectionArguments {
    $arguments = @('-c', 'port=SWD', 'mode=HOTPLUG', "freq=$script:ActiveFrequencyKhz")
    if ($script:SharedConnectionValue) {
        $arguments += 'shared'
    }
    if ($script:SelectedProbeSN) {
        $arguments += "sn=$script:SelectedProbeSN"
    }
    return $arguments
}

function New-HotPlugArguments {
    param(
        [Parameter(Mandatory = $true)][string]$Address,
        [Parameter(Mandatory = $true)][long]$Size,
        [Parameter(Mandatory = $true)][string]$Destination
    )
    $arguments = @(Get-HotPlugConnectionArguments)
    $arguments += @('--upload', $Address, [string]$Size, $Destination)
    return $arguments
}

function Invoke-StatusSnapshot {
    param([ValidateSet('preflight', 'post-export')][string]$SnapshotTag = 'preflight')
    $script:Stage = "read-status-$SnapshotTag"
    $script:DiagnosticFailure = $null
    $script:StatusSnapshotPath = $null
    if ($SnapshotTag -eq 'preflight') { $script:StatusPreflightPath = $null }
    if ($SnapshotTag -eq 'post-export') { $script:StatusPostExportPath = $null }
    $statusScript = Join-Path $PSScriptRoot 'read_run_status.ps1'
    $diagnosticRoot = Join-Path $script:RunDirectory ("status-read-{0}" -f $SnapshotTag)
    $statusJson = Join-Path $script:RunDirectory ("status-{0}.json" -f $SnapshotTag)
    $tagSnapshotPath = $statusJson
    $logPath = Join-Path $script:RunDirectory ("status-read-{0}-wrapper.log" -f $SnapshotTag)
    try {
        if (-not (Test-Path -LiteralPath $statusScript -PathType Leaf)) {
            throw "Status reader wrapper is missing: $statusScript"
        }
        $powershellPath = (Get-Process -Id $PID -ErrorAction Stop).Path
        $arguments = @(
            '-NoLogo', '-NoProfile', '-NonInteractive', '-ExecutionPolicy', 'Bypass',
            '-File', $statusScript,
            '-Elf', [IO.Path]::GetFullPath($Elf),
            '-OutDirectory', $diagnosticRoot,
            '-ProgrammerCLI', $script:ProgrammerPath,
            '-Python', $script:PythonExe,
            '-ConnectionFrequencyKhz', [string]$script:ActiveFrequencyKhz,
            '-SharedConnection', [string][bool]$script:SharedConnectionValue,
            '-NoFrequencyFallback'
        )
        if ($script:SelectedProbeSN) { $arguments += @('-ProbeSN', $script:SelectedProbeSN) }

        $previousPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            $output = @(& $powershellPath @arguments 2>&1)
            $exitCode = $LASTEXITCODE
        }
        finally { $ErrorActionPreference = $previousPreference }
        $lines = @($output | ForEach-Object { $_.ToString() })
        Set-Content -LiteralPath $logPath -Value ($lines -join [Environment]::NewLine) -Encoding UTF8

        $candidate = Get-ChildItem -LiteralPath $diagnosticRoot -Directory -ErrorAction SilentlyContinue |
            Sort-Object -Property LastWriteTime -Descending | Select-Object -First 1
        $nestedStatus = if ($candidate) { Join-Path $candidate.FullName 'run-status.json' } else { $null }
        if ($nestedStatus -and (Test-Path -LiteralPath $nestedStatus -PathType Leaf)) {
            Copy-Item -LiteralPath $nestedStatus -Destination $tagSnapshotPath -Force
            $script:StatusSnapshotPath = $tagSnapshotPath
            if ($SnapshotTag -eq 'preflight') { $script:StatusPreflightPath = $tagSnapshotPath }
            if ($SnapshotTag -eq 'post-export') { $script:StatusPostExportPath = $tagSnapshotPath }
        }
        if ($exitCode -ne 0) {
            $script:DiagnosticFailure = "read_run_status.ps1 exited with code $exitCode. See $logPath."
        }
        if (-not (Test-Path -LiteralPath $tagSnapshotPath -PathType Leaf)) {
            $script:DiagnosticFailure = "Status reader did not create run-status.json. See $logPath."
        }
    }
    catch {
        $script:DiagnosticFailure = $_.Exception.Message
        try { Set-Content -LiteralPath $logPath -Value $script:DiagnosticFailure -Encoding UTF8 } catch { }
    }
}

function Test-StatusMatchesHeader {
    param(
        [Parameter(Mandatory = $true)][object]$Status,
        [Parameter(Mandatory = $true)][object]$ExpectedHeader,
        [Parameter(Mandatory = $true)][string]$Tag
    )
    $class = [string]$Status.status_class
    if ($class -notin @('frozen', 'legacy_frozen')) {
        return "${Tag} status class is '$class', not a stable FROZEN state."
    }
    foreach ($headerName in @('runtime_log_header', 'runtime_log_header_after')) {
        $statusHeader = $Status.$headerName
        if (-not $statusHeader -or
            [int]$statusHeader.state -ne [int]$ExpectedHeader.state -or
            [int]$statusHeader.count -ne [int]$ExpectedHeader.count -or
            [uint32]$statusHeader.session_id -ne [uint32]$ExpectedHeader.session_id -or
            [string]$statusHeader.build_id -ne [string]$ExpectedHeader.build_id) {
            return "${Tag} ${headerName} does not match the export header state/count/session/build ID."
        }
    }
    $diagnostic = $Status.runtime_diagnostic_snapshot
    if ($diagnostic) {
        if ([int]$diagnostic.cpu_header_state -ne [int]$ExpectedHeader.state -or
            [int]$diagnostic.cpu_header_count -ne [int]$ExpectedHeader.count -or
            [uint32]$diagnostic.header_session_id -ne [uint32]$ExpectedHeader.session_id) {
            return "${Tag} CPU diagnostic snapshot does not match the export header state/count/session."
        }
    } elseif ($class -notin @('legacy_frozen')) {
        return "${Tag} diagnostic snapshot is missing for a non-legacy ELF."
    }
    $speedPiDiagnostic = $Status.speed_pi_diagnostic_snapshot
    if ($speedPiDiagnostic -and [int]$speedPiDiagnostic.pi_type -ne 0) {
        if (-not $speedPiDiagnostic.frozen -or
            -not $speedPiDiagnostic.start_token_matches_runtime_diagnostic) {
            return "${Tag} enabled speed PI snapshot is not frozen and linked to the runtime PE7 start token."
        }
    }
    return $null
}

function Save-SpeedPiDiagnosticArtifact {
    param([Parameter(Mandatory = $true)][string]$StatusPath,
          [Parameter(Mandatory = $true)][ValidateSet('preflight', 'post-export')][string]$Tag)
    $status = Get-Content -LiteralPath $StatusPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $snapshotProperty = $status.PSObject.Properties['speed_pi_diagnostic_snapshot']
    if (-not $snapshotProperty -or $null -eq $snapshotProperty.Value) {
        return $null
    }
    $snapshot = $snapshotProperty.Value
    $rawProperty = $status.PSObject.Properties['raw_files']
    $rawEntry = if ($rawProperty) { $rawProperty.Value.speed_pi_diagnostic_snapshot } else { $null }
    if (-not $rawEntry -or -not $rawEntry.path -or
        -not (Test-Path -LiteralPath $rawEntry.path -PathType Leaf)) {
        throw "${Tag} status contains a decoded speed PI snapshot without its retained raw binary."
    }
    $destination = Join-Path $script:RunDirectory ("speed-pi-diagnostic-{0}.bin" -f $Tag)
    Copy-Item -LiteralPath $rawEntry.path -Destination $destination -Force
    if ((Get-Item -LiteralPath $destination).Length -ne 128) {
        throw "${Tag} speed PI diagnostic binary is not exactly 128 bytes."
    }
    $copiedHash = (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($rawEntry.sha256 -and $copiedHash -ne [string]$rawEntry.sha256) {
        throw "${Tag} speed PI diagnostic binary differs from the status reader's validated upload."
    }
    if ($Tag -eq 'preflight') {
        $script:SpeedPiDiagnosticPreflight = $snapshot
        $script:SpeedPiDiagnosticPreflightRaw = $destination
    } else {
        $script:SpeedPiDiagnosticPostExport = $snapshot
        $script:SpeedPiDiagnosticPostExportRaw = $destination
    }
    return $destination
}

function Test-SpeedPiDiagnosticStable {
    if ($null -eq $script:SpeedPiDiagnosticPreflight -and
        $null -eq $script:SpeedPiDiagnosticPostExport) { return $null }
    if ($null -eq $script:SpeedPiDiagnosticPreflight -or
        $null -eq $script:SpeedPiDiagnosticPostExport) {
        return 'Speed PI diagnostic snapshot is missing from one side of the export bracket.'
    }
    if (-not $script:SpeedPiDiagnosticPreflightRaw -or
        -not $script:SpeedPiDiagnosticPostExportRaw -or
        -not (Test-Path -LiteralPath $script:SpeedPiDiagnosticPreflightRaw -PathType Leaf) -or
        -not (Test-Path -LiteralPath $script:SpeedPiDiagnosticPostExportRaw -PathType Leaf)) {
        return 'Speed PI diagnostic raw binary is missing from the export bracket.'
    }
    $preHash = (Get-FileHash -LiteralPath $script:SpeedPiDiagnosticPreflightRaw -Algorithm SHA256).Hash
    $postHash = (Get-FileHash -LiteralPath $script:SpeedPiDiagnosticPostExportRaw -Algorithm SHA256).Hash
    if ($preHash -ne $postHash) {
        return 'Frozen speed PI diagnostic snapshot changed during run-log export.'
    }
    return $null
}

function Save-SpeedPiTraceHeaderArtifact {
    param([Parameter(Mandatory = $true)][string]$StatusPath,
          [Parameter(Mandatory = $true)][ValidateSet('preflight', 'post-export')][string]$Tag)
    $status = Get-Content -LiteralPath $StatusPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $headerProperty = $status.PSObject.Properties['speed_pi_trace_header']
    if (-not $headerProperty -or $null -eq $headerProperty.Value) { return $null }
    $snapshot = $headerProperty.Value
    $rawFiles = $status.raw_files
    $rawEntry = if ($rawFiles) { $rawFiles.speed_pi_trace_header } else { $null }
    if (-not $rawEntry -or -not $rawEntry.path -or
        -not (Test-Path -LiteralPath $rawEntry.path -PathType Leaf)) {
        throw "${Tag} status contains a decoded PI trace header without its retained raw binary."
    }
    $destination = Join-Path $script:RunDirectory ("speed-pi-trace-header-{0}.bin" -f $Tag)
    Copy-Item -LiteralPath $rawEntry.path -Destination $destination -Force
    if ((Get-Item -LiteralPath $destination).Length -ne 64) {
        throw "${Tag} speed PI trace header is not exactly 64 bytes."
    }
    $hash = (Get-FileHash -LiteralPath $destination -Algorithm SHA256).Hash.ToLowerInvariant()
    if ($rawEntry.sha256 -and $hash -ne [string]$rawEntry.sha256) {
        throw "${Tag} speed PI trace header differs from the status reader's validated upload."
    }
    if ($Tag -eq 'preflight') {
        $script:SpeedPiTracePreflight = $snapshot
        $script:SpeedPiTracePreflightHeaderRaw = $destination
        $rx = if ($rawFiles) { $rawFiles.receive_diagnostic_snapshot } else { $null }
        if ($rx -and $rx.path -and (Test-Path -LiteralPath $rx.path -PathType Leaf)) {
            $script:ReceiveDiagnosticPreflightRaw = $rx.path
        }
    } else {
        $script:SpeedPiTracePostExport = $snapshot
        $script:SpeedPiTracePostExportHeaderRaw = $destination
    }
    return $destination
}

function Test-SpeedPiTraceStable {
    if ($null -eq $script:SpeedPiTracePreflight -and
        $null -eq $script:SpeedPiTracePostExport) { return $null }
    if ($null -eq $script:SpeedPiTracePreflight -or
        $null -eq $script:SpeedPiTracePostExport) {
        return 'Speed PI trace header is missing from one side of the export bracket.'
    }
    if ($script:SpeedPiTracePreflight.state_name -eq 'idle' -and
        $script:SpeedPiTracePostExport.state_name -eq 'idle') {
        $script:SpeedPiTraceExportStatus = 'idle-no-trace'
        return $null
    }
    if ($script:SpeedPiTracePreflight.state_name -ne 'frozen' -or
        $script:SpeedPiTracePostExport.state_name -ne 'frozen') {
        return 'Speed PI trace was not FROZEN in both status snapshots.'
    }
    foreach ($path in @($script:SpeedPiTracePreflightHeaderRaw,
                        $script:SpeedPiTracePostExportHeaderRaw)) {
        if (-not $path -or -not (Test-Path -LiteralPath $path -PathType Leaf)) {
            return 'Speed PI trace header raw binary is missing from the export bracket.'
        }
    }
    $pre = (Get-FileHash -LiteralPath $script:SpeedPiTracePreflightHeaderRaw -Algorithm SHA256).Hash
    $post = (Get-FileHash -LiteralPath $script:SpeedPiTracePostExportHeaderRaw -Algorithm SHA256).Hash
    if ($pre -ne $post) { return 'Frozen speed PI trace header changed during run-log export.' }
    if (-not $script:SpeedPiTraceRawPath -or
        -not (Test-Path -LiteralPath $script:SpeedPiTraceRawPath -PathType Leaf)) {
        return 'Full frozen speed PI trace upload is missing.'
    }
    $traceBytes = [IO.File]::ReadAllBytes($script:SpeedPiTraceRawPath)
    $preHeaderBytes = [IO.File]::ReadAllBytes($script:SpeedPiTracePreflightHeaderRaw)
    if ($traceBytes.Length -lt 64 -or
        [Convert]::ToBase64String($traceBytes[0..63]) -ne
        [Convert]::ToBase64String($preHeaderBytes)) {
        return 'Full trace upload header differs from the validated frozen preflight header.'
    }
    return $null
}

function Export-SpeedPiTrace {
    if (-not $script:SpeedPiTracePreflight) { return $null }
    if ($script:SpeedPiTracePreflight.state_name -eq 'idle') {
        $script:SpeedPiTraceExportStatus = 'idle-no-trace'
        return $null
    }
    $status = Get-Content -LiteralPath $script:StatusPreflightPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $rawFiles = $status.raw_files
    $piRaw = if ($script:SpeedPiDiagnosticPreflightRaw) { $script:SpeedPiDiagnosticPreflightRaw } else { $null }
    $runtimeRaw = if ($rawFiles.runtime_diagnostic_snapshot) { $rawFiles.runtime_diagnostic_snapshot.path } else { $null }
    $receiveRaw = $script:ReceiveDiagnosticPreflightRaw
    $script:SpeedPiTraceCsvPath = Join-Path $script:RunDirectory 'speed-pi-trace.csv'
    $script:SpeedPiTraceMetadataPath = Join-Path $script:RunDirectory 'speed-pi-trace.json'
    $traceExporter = Join-Path $PSScriptRoot 'speed_pi_trace_export.py'
    $arguments = @($traceExporter, '--elf', [IO.Path]::GetFullPath($Elf),
        '--raw', $script:SpeedPiTraceRawPath, '--out-csv', $script:SpeedPiTraceCsvPath,
        '--metadata', $script:SpeedPiTraceMetadataPath)
    if ($piRaw) { $arguments += @('--pi-diagnostic', $piRaw) }
    if ($runtimeRaw) { $arguments += @('--runtime-diagnostic', $runtimeRaw) }
    if ($receiveRaw) { $arguments += @('--receive-diagnostic', $receiveRaw) }
    try {
        $traceMetadata = Invoke-PythonJson -Arguments $arguments `
            -LogPath (Join-Path $script:RunDirectory 'python-speed-pi-trace-export.log')
        if (-not (Test-Path -LiteralPath $script:SpeedPiTraceCsvPath -PathType Leaf) -or
            (Get-Item -LiteralPath $script:SpeedPiTraceCsvPath).Length -le 0 -or
            -not (Test-Path -LiteralPath $script:SpeedPiTraceMetadataPath -PathType Leaf) -or
            [int]$traceMetadata.sample_count -le 0) {
            throw 'Speed PI trace exporter did not produce a nonempty validated CSV and metadata.'
        }
        $script:SpeedPiTraceExportStatus = 'verified'
        return $script:SpeedPiTraceMetadataPath
    } catch {
        $script:SpeedPiTraceExportStatus = 'failed'
        throw
    }
}

function Write-SpeedPiDiagnosticSummary {
    if ($null -eq $script:SpeedPiDiagnosticPreflight -and
        $null -eq $script:SpeedPiDiagnosticPostExport) { return $null }
    $script:SpeedPiDiagnosticJsonPath = Join-Path $script:RunDirectory 'speed-pi-diagnostic.json'
    $summary = [ordered]@{
        association = 'manual PE7 start token; checked against runtime diagnostic token'
        selected_elf = if ($layout) { $layout.elf_path } else { [IO.Path]::GetFullPath($Elf) }
        preflight_raw_binary = $script:SpeedPiDiagnosticPreflightRaw
        post_export_raw_binary = $script:SpeedPiDiagnosticPostExportRaw
        preflight = $script:SpeedPiDiagnosticPreflight
        post_export = $script:SpeedPiDiagnosticPostExport
        byte_identical_across_export = if ($script:SpeedPiDiagnosticPreflightRaw -and
            $script:SpeedPiDiagnosticPostExportRaw) {
                (Get-FileHash -LiteralPath $script:SpeedPiDiagnosticPreflightRaw -Algorithm SHA256).Hash -eq
                (Get-FileHash -LiteralPath $script:SpeedPiDiagnosticPostExportRaw -Algorithm SHA256).Hash
            } else { $false }
    }
    $summary | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath $script:SpeedPiDiagnosticJsonPath -Encoding UTF8
    return $script:SpeedPiDiagnosticJsonPath
}

function Invoke-HotPlugUpload {
    param(
        [Parameter(Mandatory = $true)][string]$Tag,
        [Parameter(Mandatory = $true)][string]$Address,
        [Parameter(Mandatory = $true)][long]$Size,
        [Parameter(Mandatory = $true)][string]$Destination
    )
    $frequencies = @(Get-ConnectionFrequencies)
    for ($index = 0; $index -lt $frequencies.Count; ++$index) {
        $script:ActiveFrequencyKhz = [int]$frequencies[$index]
        $arguments = New-HotPlugArguments -Address $Address -Size $Size -Destination $Destination
        $attemptNumber = $index + 1
        $logPath = Join-Path $script:RunDirectory ("cubeprogrammer-{0}-attempt-{1:D2}-{2}.log" -f $Tag, $attemptNumber, $script:ActiveFrequencyKhz)
        $script:CliLogs.Add($logPath)
        $script:Stage = "hotplug-$Tag"
        $exitCode = $null
        $text = ''
        $previousPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            $output = @(& $script:ProgrammerPath @arguments 2>&1)
            $exitCode = $LASTEXITCODE
            $text = (($output | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine)
        } catch {
            $text = $_.Exception.Message
        } finally { $ErrorActionPreference = $previousPreference }
        Set-Content -LiteralPath $logPath -Value $text -Encoding UTF8
        $actualFrequency = $null
        if ($text -match '(?im)^\s*SWD\s+freq\s*:\s*(\d+)\s*KHz\s*$') {
            $actualFrequency = [int]$Matches[1]
        }
        $attemptErrorClass = if ($exitCode -eq 0) { $null } else {
            Resolve-ToolErrorClass -Message $text -Stage $script:Stage
        }
        $attempt = [ordered]@{
            tag = $Tag
            attempt = $attemptNumber
            requested_frequency_khz = $script:ActiveFrequencyKhz
            actual_frequency_khz = $actualFrequency
            shared_connection = [bool]$script:SharedConnectionValue
            probe_sn = $script:SelectedProbeSN
            arguments = $arguments
            exit_code = $exitCode
            error_class = $attemptErrorClass
            full_output = $text
            log = $logPath
        }
        $script:ConnectionAttempts.Add($attempt)
        Save-ConnectionAttempts
        if ($exitCode -eq 0) {
            if (-not (Test-Path -LiteralPath $Destination -PathType Leaf)) {
                $script:ErrorClass = 'ram_read'
                $attempt.error_class = $script:ErrorClass
                Save-ConnectionAttempts
                throw "STM32CubeProgrammer reported success but did not create $Destination. See $logPath."
            }
            $actualSize = (Get-Item -LiteralPath $Destination).Length
            if ($actualSize -ne $Size) {
                $script:ErrorClass = 'ram_read'
                $attempt.error_class = $script:ErrorClass
                Save-ConnectionAttempts
                throw "STM32CubeProgrammer wrote $actualSize bytes for a $Size-byte $Tag upload. See $logPath."
            }
            if ($actualFrequency) { $script:ActualFrequencyKhz = $actualFrequency }
            $script:Connected = $true
            return
        }
        if ($attemptErrorClass -eq 'core_inaccessible' -and -not $script:Connected -and
            $index + 1 -lt $frequencies.Count) {
            continue
        }
        $script:ErrorClass = $attemptErrorClass
        throw "STM32CubeProgrammer failed during $Tag upload (exit $exitCode; error_class=$attemptErrorClass). See $logPath. $text"
    }
}

function Get-RunMetadata {
    param([string]$Status, [string]$ErrorMessage, [object]$Layout, [object]$Header)
    $rawFiles = @()
    if ($script:RunDirectory -and (Test-Path -LiteralPath $script:RunDirectory -PathType Container)) {
        foreach ($name in @('flash-rom-descriptor.bin', 'header-initial.bin', 'runtime-log.bin', 'header-final.bin', 'status-preflight.json', 'status-post-export.json', 'speed-pi-diagnostic-preflight.bin', 'speed-pi-diagnostic-post-export.bin', 'speed-pi-diagnostic.json', 'speed-pi-trace-header-preflight.bin', 'speed-pi-trace-header-post-export.bin', 'speed-pi-trace.bin', 'speed-pi-trace.csv', 'speed-pi-trace.json')) {
            $path = Join-Path $script:RunDirectory $name
            if (Test-Path -LiteralPath $path -PathType Leaf) {
                $rawFiles += [ordered]@{ path = $path; bytes = (Get-Item -LiteralPath $path).Length }
            }
        }
    }
    $rawPath = if ($script:RunDirectory) { Join-Path $script:RunDirectory 'runtime-log.bin' } else { $null }
    $rawHash = if ($rawPath -and (Test-Path -LiteralPath $rawPath -PathType Leaf)) {
        (Get-FileHash -LiteralPath $rawPath -Algorithm SHA256).Hash.ToLowerInvariant()
    } else { $null }
    return [ordered]@{
        status = $Status
        stage = $script:Stage
        started_at_utc = $script:StartedAtUtc
        finished_at_utc = [DateTime]::UtcNow.ToString('o')
        error = $ErrorMessage
        error_class = $script:ErrorClass
        elf = if ($Layout) { $Layout.elf_path } else { [IO.Path]::GetFullPath($Elf) }
        elf_sha256 = if ($Layout) { $Layout.elf_sha256 } else { $null }
        storage_address = if ($Layout) { $Layout.storage_address } else { $null }
        storage_size = if ($Layout) { $Layout.storage_size } else { $null }
        build_id = if ($Layout) { $Layout.build_id } else { $null }
        log_version = if ($Layout) { [int]$Layout.descriptor.log_version } else { $null }
        holdQuantum_ms = if ($Layout) {
            if ([int]$Layout.descriptor.log_version -eq 1) { 20 } else { 50 }
        } else { $null }
        probe_serial_supplied = [bool]$ProbeSN
        probe_sn = $script:SelectedProbeSN
        requested_connection_frequency_khz = $script:StartingFrequencyKhz
        connection_frequency_khz = $script:ActiveFrequencyKhz
        actual_connection_frequency_khz = $script:ActualFrequencyKhz
        shared_connection = [bool]$script:SharedConnectionValue
        connection = "SWD HOTPLUG freq=$($script:ActiveFrequencyKhz)$(if ($script:SharedConnectionValue) { ' shared' })$(if ($script:SelectedProbeSN) { ' sn=' + $script:SelectedProbeSN })"
        connection_attempts = $script:ConnectionAttempts.ToArray()
        flash_descriptor = $script:FlashDescriptorValidation
        initial_header = $Header
        record_count = if ($Header) { [int]$Header.count } else { $null }
        dropped_count = if ($Header) { [int]$Header.dropped_count } else { $null }
        buffer_full = if ($Header -and $Header.PSObject.Properties['buffer_full']) { [bool]$Header.buffer_full } else { $null }
        records_truncated = if ($Header -and $Header.PSObject.Properties['records_truncated']) { [bool]$Header.records_truncated } else { $null }
        csv = if ($script:RunCsvValidated -and $script:RunCsvPath -and
                  (Test-Path -LiteralPath $script:RunCsvPath -PathType Leaf)) {
            $script:RunCsvPath
        } else { $null }
        planned_csv_path = $script:RunCsvPath
        status_snapshot = if ($script:StatusSnapshotPath) { $script:StatusSnapshotPath } else { $null }
        status_preflight = if ($script:StatusPreflightPath) { $script:StatusPreflightPath } else { $null }
        status_post_export = if ($script:StatusPostExportPath) { $script:StatusPostExportPath } else { $null }
        speed_pi_diagnostic_json = $script:SpeedPiDiagnosticJsonPath
        speed_pi_diagnostic_preflight = $script:SpeedPiDiagnosticPreflight
        speed_pi_diagnostic_post_export = $script:SpeedPiDiagnosticPostExport
        speed_pi_trace_status = $script:SpeedPiTraceExportStatus
        speed_pi_trace_preflight = $script:SpeedPiTracePreflight
        speed_pi_trace_post_export = $script:SpeedPiTracePostExport
        speed_pi_trace_raw = if ($script:SpeedPiTraceRawPath -and (Test-Path -LiteralPath $script:SpeedPiTraceRawPath -PathType Leaf)) { $script:SpeedPiTraceRawPath } else { $null }
        speed_pi_trace_csv = if ($script:SpeedPiTraceExportStatus -eq 'verified') { $script:SpeedPiTraceCsvPath } else { $null }
        speed_pi_trace_metadata = if ($script:SpeedPiTraceExportStatus -eq 'verified') { $script:SpeedPiTraceMetadataPath } else { $null }
        run_state_class = $script:RunStateClass
        export_diagnostic_status = $script:ExportDiagnosticStatus
        diagnostic_failure = $script:DiagnosticFailure
        raw_sha256 = $rawHash
        cubeprogrammer_logs = @($script:CliLogs)
        raw_files = $rawFiles
    }
}

$layout = $null
$initialHeader = $null

try {
    if (-not (Test-Path -LiteralPath $Elf -PathType Leaf)) {
        throw "ELF file does not exist: $Elf"
    }
    $elfPath = (Resolve-Path -LiteralPath $Elf).Path
    if ((Split-Path -Leaf $elfPath) -ne 'CarTest.elf') {
        throw 'Pass the selected CarTest.elf explicitly; similarly named projects are not accepted.'
    }

    if ($OutDirectory) {
        $outRoot = [IO.Path]::GetFullPath($OutDirectory)
    }
    else {
        $outRoot = Join-Path $script:ProjectDirectory 'logs\runs'
    }
    $timestamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
    $runName = "{0}-{1}" -f $timestamp, $PID
    $script:RunDirectory = Join-Path $outRoot $runName
    $script:RunCsvPath = Join-Path $script:RunDirectory 'run.csv'
    $script:DecodeMetadataPath = Join-Path $script:RunDirectory 'decode-metadata.json'

    $script:PythonExe = Resolve-PythonExe
    $validator = Join-Path $PSScriptRoot 'runtime_log_export.py'
    if (-not (Test-Path -LiteralPath $validator -PathType Leaf)) {
        throw "ELF/log validator is missing: $validator"
    }

    New-Item -ItemType Directory -Path $script:RunDirectory -Force | Out-Null
    $script:Stage = 'inspect-selected-elf'
    $layoutLog = Join-Path $script:RunDirectory 'python-inspect-elf.log'
    $layout = Invoke-PythonJson -Arguments @($validator, 'inspect', '--elf', $elfPath) -LogPath $layoutLog
    $storageAddress = [string]$layout.storage_address
    $storageSize = [long]$layout.storage_size
    $headerAddress = $storageAddress

    if ($DryRun) {
        $cliPath = if ($ProgrammerCLI) { [IO.Path]::GetFullPath($ProgrammerCLI) } else { '<auto-detected when run>' }
        $script:ActiveFrequencyKhz = $script:StartingFrequencyKhz
        $script:SelectedProbeSN = if ($ProbeSN) { $ProbeSN } else { '<unique probe from -l st-link-only>' }
        $sampleArgs = New-HotPlugArguments -Address $headerAddress -Size 64 -Destination '<run-directory>\header-initial.bin'
        $secondRead = @(Get-HotPlugConnectionArguments) + @('--upload', $storageAddress, '<64 + frozen_count * 64>', '<run-directory>\runtime-log.bin')
        $finalRead = @(Get-HotPlugConnectionArguments) + @('--upload', $storageAddress, '64', '<run-directory>\header-final.bin')
        $statusReader = Join-Path $PSScriptRoot 'run_status_reader.py'
        $statusPlan = Invoke-PythonJson -Arguments @($statusReader, 'plan', '--elf', $elfPath) `
            -LogPath (Join-Path $script:RunDirectory 'python-status-plan.log')
        $statusScalarRead = New-HotPlugArguments -Address $statusPlan.scalar_read.address `
            -Size ([long]$statusPlan.scalar_read.size) -Destination '<run-directory>\status-read\scalar-region.bin'
        $statusHeaderRead = New-HotPlugArguments -Address $statusPlan.header_read.address `
            -Size 64 -Destination '<run-directory>\status-read\runtime-log-header.bin'
        $receiveDiagnosticRead = if ($statusPlan.receive_diagnostic_snapshot.available) {
            New-HotPlugArguments -Address $statusPlan.receive_diagnostic_snapshot.address -Size 96 -Destination '<run-directory>\status-read\receive-diagnostic.bin'
        } else { $null }
        $speedPiTraceHeaderRead = if ($statusPlan.speed_pi_trace_snapshot.available) {
            New-HotPlugArguments -Address $statusPlan.speed_pi_trace_snapshot.address -Size 64 -Destination '<run-directory>\status-read\speed-pi-trace-header.bin'
        } else { $null }
        $speedPiTraceFullRead = if ($statusPlan.speed_pi_trace_snapshot.available) {
            New-HotPlugArguments -Address $statusPlan.speed_pi_trace_snapshot.address -Size ([long]$statusPlan.speed_pi_trace_snapshot.size) -Destination '<run-directory>\speed-pi-trace.bin'
        } else { $null }
        $descriptorRead = New-HotPlugArguments -Address $layout.descriptor_address `
            -Size 64 -Destination '<run-directory>\flash-rom-descriptor.bin'
        $plan = [ordered]@{
            dry_run = $true
            selected_elf = $layout.elf_path
            elf_sha256 = $layout.elf_sha256
            build_id = $layout.build_id
            programmer_cli = $cliPath
            probe_enumeration_arguments = @('-l', 'st-link-only')
            frequency_attempts_khz = @(Get-ConnectionFrequencies)
            requested_connection_frequency_khz = $script:StartingFrequencyKhz
            connection_frequency_khz = $script:ActiveFrequencyKhz
            shared_connection = [bool]$script:SharedConnectionValue
            probe_serial_supplied = [bool]$ProbeSN
            flash_descriptor_read = $descriptorRead
            ram_address = $storageAddress
            ram_size = $storageSize
            first_read = $sampleArgs
            second_read = $secondRead
            final_read = $finalRead
            status_snapshot = [ordered]@{
                selected_elf = $statusPlan.elf_path
                log_version = $statusPlan.log_version
                hold_quantum_ms = $statusPlan.hold_quantum_ms
                scalar_symbols = $statusPlan.scalar_symbols
                descriptor_read = $descriptorRead
                scalar_read = $statusScalarRead
                header_read = $statusHeaderRead
                speed_pi_diagnostic_snapshot = $statusPlan.speed_pi_diagnostic_snapshot
                receive_diagnostic_snapshot = $statusPlan.receive_diagnostic_snapshot
                speed_pi_trace_header_read = $speedPiTraceHeaderRead
                speed_pi_trace_full_read = $speedPiTraceFullRead
                record_area_read = $false
            }
            output_directory = $script:RunDirectory
        }
        $plan | ConvertTo-Json -Depth 5
        return
    }

    $script:ProgrammerPath = Resolve-ProgrammerCli
    $script:Stage = 'enumerate-probe'
    Resolve-ProbeSerial
    $probeArguments = New-HotPlugArguments -Address $headerAddress -Size 64 -Destination 'x'
    $forbiddenArguments = @('mode=NORMAL', '-rst', '-halt', '--download', '--erase', '-go', '--run')
    foreach ($argument in $probeArguments) {
        if ($forbiddenArguments -contains $argument) {
            throw 'Internal guard rejected a command outside the Hot Plug upload allowlist.'
        }
    }

    $descriptorPath = Join-Path $script:RunDirectory 'flash-rom-descriptor.bin'
    $script:Stage = 'read-flash-descriptor'
    Invoke-HotPlugUpload -Tag 'flash-descriptor' -Address ([string]$layout.descriptor_address) `
        -Size 64 -Destination $descriptorPath
    $script:Stage = 'validate-flash-descriptor'
    $script:FlashDescriptorValidation = Invoke-PythonJson -Arguments @(
        $validator, 'check-descriptor', '--elf', $elfPath, '--descriptor', $descriptorPath
    ) -LogPath (Join-Path $script:RunDirectory 'python-check-flash-descriptor.log')

    Invoke-StatusSnapshot -SnapshotTag 'preflight'
    if ($script:DiagnosticFailure -or -not $script:StatusPreflightPath) {
        $script:ErrorClass = 'diagnostic_preflight'
        throw "Could not obtain a coherent runtime diagnostic preflight. $($script:DiagnosticFailure)"
    }
    $preflightStatus = Get-Content -LiteralPath $script:StatusPreflightPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $null = Save-SpeedPiDiagnosticArtifact -StatusPath $script:StatusPreflightPath -Tag 'preflight'
    $null = Save-SpeedPiTraceHeaderArtifact -StatusPath $script:StatusPreflightPath -Tag 'preflight'
    if ($script:SpeedPiTracePreflight -and
        $script:SpeedPiTracePreflight.state_name -eq 'idle') {
        $script:SpeedPiTraceExportStatus = 'idle-no-trace'
    } elseif ($script:SpeedPiTracePreflight -and
              $script:SpeedPiTracePreflight.state_name -eq 'active') {
        $script:SpeedPiTraceExportStatus = 'active-no-export'
    }
    $null = Write-SpeedPiDiagnosticSummary
    $script:RunStateClass = [string]$preflightStatus.status_class
    $preflightHeader = $preflightStatus.runtime_log_header_after
    if ($script:RunStateClass -in @('neverstarted', 'legacy_idle', 'active', 'legacy_active')) {
        $script:Stage = "preflight-$($script:RunStateClass)-no-run-export"
        $initialHeader = $preflightHeader
        $message = if ($script:RunStateClass -in @('neverstarted', 'legacy_idle')) {
            'Runtime log is IDLE; no run records are available and no run.csv was created.'
        } else {
            'Runtime log is ACTIVE; stop the vehicle through its normal stop path and wait for FROZEN before export.'
        }
        $metadata = Get-RunMetadata -Status $script:RunStateClass -ErrorMessage $message -Layout $layout -Header $initialHeader
        $metadata | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-metadata.json') -Encoding UTF8
        [Console]::Error.WriteLine($message)
        [Console]::Error.WriteLine("Status snapshot: $script:StatusPreflightPath")
        exit 2
    }
    if ($script:RunStateClass -in @('inconsistent', 'legacy_inconsistent')) {
        $script:ErrorClass = 'inconsistent_snapshot'
        throw "Runtime diagnostic/header comparison remained inconsistent after bounded retries. See $script:StatusPreflightPath."
    }
    if ($script:RunStateClass -notin @('frozen', 'legacy_frozen')) {
        $script:ErrorClass = 'diagnostic_preflight'
        throw "Unsupported runtime diagnostic status class '$($script:RunStateClass)'; export was not started."
    }

    $initialHeaderPath = Join-Path $script:RunDirectory 'header-initial.bin'
    $script:Stage = 'read-initial-header'
    Invoke-HotPlugUpload -Tag 'header-initial' -Address $headerAddress -Size 64 -Destination $initialHeaderPath
    $script:Stage = 'validate-initial-header'
    $initialHeader = Invoke-PythonJson -Arguments @($validator, 'check-header', '--elf', $elfPath,
        '--header', $initialHeaderPath, '--allow-idle') -LogPath (Join-Path $script:RunDirectory 'python-check-initial-header.log')

    if ([int]$initialHeader.state -eq 0) {
        $script:Stage = 'preflight-header-disagreement'
        $script:RunStateClass = 'inconsistent'
        $idleMessage = 'The log became IDLE after a FROZEN preflight. No run.csv was created; inspect the retained status and header files.'
        $metadata = Get-RunMetadata -Status 'inconsistent' -ErrorMessage $idleMessage -Layout $layout -Header $initialHeader
        $metadata | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-metadata.json') -Encoding UTF8
        [Console]::Error.WriteLine($idleMessage)
        [Console]::Error.WriteLine("Status snapshot: $script:StatusPreflightPath")
        exit 2
    }

    if ([int]$initialHeader.state -ne 2 -or [int]$initialHeader.count -lt 2) {
        $script:ErrorClass = 'inconsistent_snapshot'
        throw "Preflight was FROZEN but the export header is not FROZEN with at least two records (state=$($initialHeader.state), count=$($initialHeader.count))."
    }
    $preflightMismatch = Test-StatusMatchesHeader -Status $preflightStatus `
        -ExpectedHeader $initialHeader -Tag 'Preflight'
    if ($preflightMismatch) {
        $script:ErrorClass = 'inconsistent_snapshot'
        $script:ExportDiagnosticStatus = 'inconsistent'
        throw $preflightMismatch
    }
    $script:ExportDiagnosticStatus = 'verified'

    if ($script:SpeedPiTracePreflight -and
        $script:SpeedPiTracePreflight.state_name -eq 'frozen') {
        $tracePlan = $preflightStatus.reads.speed_pi_trace_snapshot
        if (-not $tracePlan -or -not $tracePlan.available -or
            [int]$tracePlan.size -ne 98368) {
            $script:ErrorClass = 'trace_snapshot'
            throw 'Selected ELF has no valid 98,368-byte Speed PI trace RAM plan.'
        }
        $script:SpeedPiTraceRawPath = Join-Path $script:RunDirectory 'speed-pi-trace.bin'
        $script:Stage = 'read-frozen-speed-pi-trace'
        Invoke-HotPlugUpload -Tag 'speed-pi-trace' -Address ([string]$tracePlan.address) `
            -Size ([long]$tracePlan.size) -Destination $script:SpeedPiTraceRawPath
        $traceBytes = [IO.File]::ReadAllBytes($script:SpeedPiTraceRawPath)
        $preflightTraceHeaderBytes = [IO.File]::ReadAllBytes($script:SpeedPiTracePreflightHeaderRaw)
        if ($traceBytes.Length -ne [int]$tracePlan.size -or
            [Convert]::ToBase64String($traceBytes[0..63]) -ne
            [Convert]::ToBase64String($preflightTraceHeaderBytes)) {
            $script:ErrorClass = 'trace_snapshot'
            throw 'Full speed PI trace upload does not start with the validated FROZEN header.'
        }
        $script:SpeedPiTraceExportStatus = 'pending-postflight'
    }

    $payloadSize = [long]$initialHeader.payload_size
    if ($payloadSize -lt 64 -or $payloadSize -gt $storageSize) {
        throw "Frozen header requested an invalid payload size: $payloadSize."
    }
    $rawPath = Join-Path $script:RunDirectory 'runtime-log.bin'
    $script:Stage = 'read-frozen-payload'
    Invoke-HotPlugUpload -Tag 'payload' -Address $headerAddress -Size $payloadSize -Destination $rawPath

    $finalHeaderPath = Join-Path $script:RunDirectory 'header-final.bin'
    $script:Stage = 'read-final-header'
    Invoke-HotPlugUpload -Tag 'header-final' -Address $headerAddress -Size 64 -Destination $finalHeaderPath
    $script:Stage = 'validate-final-header'
    $finalHeader = Invoke-PythonJson -Arguments @($validator, 'check-header', '--elf', $elfPath, '--header', $finalHeaderPath) -LogPath (Join-Path $script:RunDirectory 'python-check-final-header.log')
    $initialBytes = [IO.File]::ReadAllBytes($initialHeaderPath)
    $finalBytes = [IO.File]::ReadAllBytes($finalHeaderPath)
    $rawBytes = [IO.File]::ReadAllBytes($rawPath)
    $rawHeader = New-Object byte[] 64
    if ($rawBytes.Length -ge 64) {
        [Array]::Copy($rawBytes, 0, $rawHeader, 0, 64)
    }
    if ($initialBytes.Length -ne 64 -or $finalBytes.Length -ne 64 -or
        [Convert]::ToBase64String($initialBytes) -ne [Convert]::ToBase64String($finalBytes) -or
        $rawBytes.Length -lt 64 -or
        [Convert]::ToBase64String($initialBytes) -ne [Convert]::ToBase64String($rawHeader)) {
        throw 'Frozen header changed during export or differs from the uploaded payload. The buffer may have been reset or a new session may have started.'
    }
    if ([int]$finalHeader.count -ne [int]$initialHeader.count -or
        [string]$finalHeader.build_id -ne [string]$initialHeader.build_id) {
        throw 'Final header differs from the validated initial frozen header.'
    }

    $script:Stage = 'decode-records-and-write-csv'
    $decodeLog = Join-Path $script:RunDirectory 'python-decode.log'
    $null = Invoke-PythonJson -Arguments @($validator, 'decode', '--elf', $elfPath, '--raw', $rawPath,
        '--out-csv', $script:RunCsvPath, '--metadata', $script:DecodeMetadataPath) -LogPath $decodeLog
    if (-not (Test-Path -LiteralPath $script:RunCsvPath -PathType Leaf) -or
        (Get-Item -LiteralPath $script:RunCsvPath).Length -le 0 -or
        -not (Test-Path -LiteralPath $script:DecodeMetadataPath -PathType Leaf)) {
        $script:ErrorClass = 'decode'
        throw 'Decoder did not produce a nonempty CSV and metadata file.'
    }
    $decodeMetadata = Get-Content -LiteralPath $script:DecodeMetadataPath -Raw -Encoding UTF8 | ConvertFrom-Json
    if ([int]$decodeMetadata.record_count -lt 2) {
        $script:ErrorClass = 'decode'
        throw "Decoder produced only $($decodeMetadata.record_count) record(s); an empty/header-only CSV cannot be reported as a successful export."
    }
    $script:RunCsvValidated = $true
    Invoke-StatusSnapshot -SnapshotTag 'post-export'
    if ($script:DiagnosticFailure) {
        $script:ExportDiagnosticStatus = 'unverified'
    } elseif (-not $script:StatusPostExportPath -or
              -not (Test-Path -LiteralPath $script:StatusPostExportPath -PathType Leaf)) {
        $script:DiagnosticFailure = 'Post-export status snapshot is missing.'
        $script:ExportDiagnosticStatus = 'unverified'
    } else {
        $postStatus = Get-Content -LiteralPath $script:StatusPostExportPath -Raw -Encoding UTF8 | ConvertFrom-Json
        $null = Save-SpeedPiDiagnosticArtifact -StatusPath $script:StatusPostExportPath -Tag 'post-export'
        $null = Save-SpeedPiTraceHeaderArtifact -StatusPath $script:StatusPostExportPath -Tag 'post-export'
        $postMismatch = Test-StatusMatchesHeader -Status $postStatus `
            -ExpectedHeader $finalHeader -Tag 'Post-export'
        if (-not $postMismatch) {
            $postMismatch = Test-SpeedPiDiagnosticStable
        }
        if (-not $postMismatch) {
            $postMismatch = Test-SpeedPiTraceStable
        }
        if ($postMismatch) {
            $script:DiagnosticFailure = $postMismatch
            $script:ExportDiagnosticStatus = 'inconsistent'
        } else {
            $script:ExportDiagnosticStatus = 'verified'
            try {
                $null = Export-SpeedPiTrace
            } catch {
                $script:DiagnosticFailure = $_.Exception.Message
                $script:ExportDiagnosticStatus = 'inconsistent'
            }
        }
    }
    $null = Write-SpeedPiDiagnosticSummary
    $script:Stage = 'complete'
    $completionStatus = if ($script:ExportDiagnosticStatus -eq 'inconsistent') {
        'decoded_csv_diagnostic_inconsistent'
    } elseif ($script:DiagnosticFailure -or $script:ExportDiagnosticStatus -ne 'verified') {
        'decoded_csv_diagnostic_unverified'
    } else { 'success' }
    $metadata = Get-RunMetadata -Status $completionStatus -ErrorMessage $script:DiagnosticFailure -Layout $layout -Header $initialHeader
    $metadata['record_count'] = [int]$decodeMetadata.record_count
    $metadata['dropped_count'] = [int]$decodeMetadata.dropped_count
    $metadata['buffer_full'] = [bool]$decodeMetadata.buffer_full
    $metadata['records_truncated'] = [bool]$decodeMetadata.records_truncated
    $metadata['speed_pi_diagnostic_json'] = $script:SpeedPiDiagnosticJsonPath
    $metadata['speed_pi_diagnostic_preflight'] = $script:SpeedPiDiagnosticPreflight
    $metadata['speed_pi_diagnostic_post_export'] = $script:SpeedPiDiagnosticPostExport
    $metadata['speed_pi_trace_status'] = $script:SpeedPiTraceExportStatus
    $metadata['speed_pi_trace_csv'] = if ($script:SpeedPiTraceExportStatus -eq 'verified') { $script:SpeedPiTraceCsvPath } else { $null }
    $metadata['speed_pi_trace_metadata'] = if ($script:SpeedPiTraceExportStatus -eq 'verified') { $script:SpeedPiTraceMetadataPath } else { $null }
    $metadata['raw_sha256'] = (Get-FileHash -LiteralPath $rawPath -Algorithm SHA256).Hash.ToLowerInvariant()
    $metadata | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-metadata.json') -Encoding UTF8
    foreach ($warning in @($script:DiagnosticFailure)) {
        if ($warning) { [Console]::Error.WriteLine("Warning: $warning") }
    }
    Write-Output $script:RunCsvPath
    if ($completionStatus -ne 'success') { exit 3 }
    exit 0
}
catch {
    $errorMessage = $_.Exception.Message
    if (-not $script:ErrorClass) {
        $script:ErrorClass = Resolve-ToolErrorClass -Message $errorMessage -Stage $script:Stage
    }
    if (-not $script:RunCsvValidated -and $script:RunCsvPath -and (Test-Path -LiteralPath $script:RunCsvPath -PathType Leaf)) {
        Remove-Item -LiteralPath $script:RunCsvPath -Force
    }
    if (-not $script:RunCsvValidated -and $script:DecodeMetadataPath -and (Test-Path -LiteralPath $script:DecodeMetadataPath -PathType Leaf)) {
        Remove-Item -LiteralPath $script:DecodeMetadataPath -Force
    }
    if ($script:RunDirectory -and (Test-Path -LiteralPath $script:RunDirectory -PathType Container)) {
        try {
            $failureStatus = if ($script:RunCsvValidated) {
                if ($script:ExportDiagnosticStatus -eq 'inconsistent') {
                    'decoded_csv_diagnostic_inconsistent'
                } else { 'decoded_csv_diagnostic_unverified' }
            } else { 'failed' }
            $metadata = Get-RunMetadata -Status $failureStatus -ErrorMessage $errorMessage -Layout $layout -Header $initialHeader
            $metadata | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-metadata.json') -Encoding UTF8
        }
        catch {
            # Preserve the primary export error if writing failure metadata itself is impossible.
        }
    }
    [Console]::Error.WriteLine($errorMessage)
    exit 1
}
