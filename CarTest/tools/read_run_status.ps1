[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Elf,
    [string]$OutDirectory,
    [string]$ProgrammerCLI,
    [ValidatePattern('^[A-Za-z0-9_-]{1,64}$')][string]$ProbeSN,
    [ValidateSet(100, 400, 1000)][int]$ConnectionFrequencyKhz = 1000,
    [ValidateSet('true', 'false', '1', '0')][string]$SharedConnection = 'true',
    [string]$Python = 'python.exe',
    [switch]$NoFrequencyFallback,
    [switch]$DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:ProjectDirectory = Split-Path -Parent $PSScriptRoot
$script:Reader = Join-Path $PSScriptRoot 'run_status_reader.py'
$script:Exporter = Join-Path $PSScriptRoot 'runtime_log_export.py'
$script:PythonExe = $null
$script:ProgrammerPath = $null
$script:RunDirectory = $null
$script:Stage = 'preflight'
$script:Plan = $null
$script:StartedAtUtc = [DateTime]::UtcNow.ToString('o')
$script:SelectedProbeSN = $null
$script:ActiveFrequencyKhz = $ConnectionFrequencyKhz
$script:ActualFrequencyKhz = $null
$script:ConnectionAttempts = New-Object System.Collections.Generic.List[object]
$script:ErrorClass = $null
$script:Connected = $false
$script:SharedConnectionValue = $SharedConnection -match '^(true|1)$'

function Resolve-PythonExe {
    $command = Get-Command -Name $Python -ErrorAction Stop
    if ($command.CommandType -ne 'Application') {
        throw 'Python must resolve to a python.exe application.'
    }
    return $command.Source
}

function Resolve-ProgrammerCli {
    if ($ProgrammerCLI) {
        $candidate = (Resolve-Path -LiteralPath $ProgrammerCLI -ErrorAction Stop).Path
        if ((Split-Path -Leaf $candidate) -ne 'STM32_Programmer_CLI.exe') {
            throw 'ProgrammerCLI must point to STM32_Programmer_CLI.exe.'
        }
        return $candidate
    }

    $bundleRoot = if ($env:LOCALAPPDATA) {
        Join-Path $env:LOCALAPPDATA 'stm32cube\bundles\programmer'
    } else { $null }
    if ($bundleRoot -and (Test-Path -LiteralPath $bundleRoot -PathType Container)) {
        $versions = @(Get-ChildItem -LiteralPath $bundleRoot -Directory -ErrorAction SilentlyContinue |
            ForEach-Object {
                $parsedVersion = $null
                if ([Version]::TryParse($_.Name, [ref]$parsedVersion)) {
                    [PSCustomObject]@{ Path = $_.FullName; Version = $parsedVersion }
                }
            } | Sort-Object -Property Version -Descending)
        foreach ($version in $versions) {
            $candidate = Join-Path $version.Path 'bin\STM32_Programmer_CLI.exe'
            if (Test-Path -LiteralPath $candidate -PathType Leaf) {
                return (Resolve-Path -LiteralPath $candidate).Path
            }
        }
    }

    $candidates = @()
    if ($env:ProgramFiles) {
        $candidates += Join-Path $env:ProgramFiles 'STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    }
    if (${env:ProgramFiles(x86)}) {
        $candidates += Join-Path ${env:ProgramFiles(x86)} 'STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    }
    if ($env:LOCALAPPDATA) {
        $candidates += Join-Path $env:LOCALAPPDATA 'Programs\STMicroelectronics\STM32Cube\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe'
    }
    foreach ($candidate in $candidates) {
        if (Test-Path -LiteralPath $candidate -PathType Leaf) {
            return (Resolve-Path -LiteralPath $candidate).Path
        }
    }
    $fromPath = Get-Command -Name 'STM32_Programmer_CLI.exe' -ErrorAction SilentlyContinue
    if ($fromPath -and (Test-Path -LiteralPath $fromPath.Source -PathType Leaf)) {
        return $fromPath.Source
    }
    throw 'STM32_Programmer_CLI.exe was not found. Install STM32CubeProgrammer or pass -ProgrammerCLI with its full path.'
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
    if ($NoFrequencyFallback) { return @($ConnectionFrequencyKhz) }
    return @(@(1000, 400, 100) | Where-Object { $_ -le $ConnectionFrequencyKhz })
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

function New-ReadDirectory {
    param([string]$Root)
    if (-not $Root) {
        $Root = Join-Path $script:ProjectDirectory 'logs\diagnostics'
    }
    $rootPath = [IO.Path]::GetFullPath($Root)
    New-Item -ItemType Directory -Path $rootPath -Force | Out-Null
    $name = '{0}-{1}' -f [DateTime]::Now.ToString('yyyyMMdd-HHmmss-fff'), $PID
    $candidate = Join-Path $rootPath $name
    $suffix = 1
    while (Test-Path -LiteralPath $candidate) {
        $candidate = Join-Path $rootPath ('{0}-{1}-{2}' -f [DateTime]::Now.ToString('yyyyMMdd-HHmmss-fff'), $PID, $suffix)
        $suffix++
    }
    New-Item -ItemType Directory -Path $candidate | Out-Null
    return $candidate
}

function Invoke-ReaderJson {
    param(
        [Parameter(Mandatory = $true)][string[]]$Arguments,
        [Parameter(Mandatory = $true)][AllowEmptyString()][string]$LogPath
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
    $stdout = New-Object System.Collections.Generic.List[string]
    $log = New-Object System.Collections.Generic.List[string]
    foreach ($line in $output) {
        if ($line -is [System.Management.Automation.ErrorRecord]) {
            $log.Add("stderr: {0}" -f $line.ToString())
        } else {
            $stdout.Add($line.ToString())
            $log.Add("stdout: {0}" -f $line.ToString())
        }
    }
    if ($LogPath) {
        Set-Content -LiteralPath $LogPath -Value ($log -join [Environment]::NewLine) -Encoding UTF8
    }
    $text = $stdout -join [Environment]::NewLine
    if ($exitCode -ne 0) {
        throw "Status reader failed (exit $exitCode). $($log -join ' ')"
    }
    try {
        return ($text | ConvertFrom-Json -ErrorAction Stop)
    } catch {
        throw "Status reader returned invalid JSON. See $LogPath."
    }
}

function Get-ReadArguments {
    param(
        [Parameter(Mandatory = $true)][string]$Address,
        [Parameter(Mandatory = $true)][long]$Size,
        [Parameter(Mandatory = $true)][string]$Destination
    )
    $arguments = @('-c', 'port=SWD', 'mode=HOTPLUG', "freq=$script:ActiveFrequencyKhz")
    if ($script:SharedConnectionValue) { $arguments += 'shared' }
    if ($script:SelectedProbeSN) { $arguments += "sn=$script:SelectedProbeSN" }
    $arguments += @('--upload', $Address, [string]$Size, $Destination)
    return $arguments
}

function Invoke-StatusRead {
    param(
        [Parameter(Mandatory = $true)][string]$Tag,
        [Parameter(Mandatory = $true)][string]$Address,
        [Parameter(Mandatory = $true)][long]$Size,
        [Parameter(Mandatory = $true)][string]$Destination
    )
    $frequencies = @(Get-ConnectionFrequencies)
    for ($index = 0; $index -lt $frequencies.Count; ++$index) {
        $script:ActiveFrequencyKhz = [int]$frequencies[$index]
        $arguments = Get-ReadArguments -Address $Address -Size $Size -Destination $Destination
        $attemptNumber = $index + 1
        $logPath = Join-Path $script:RunDirectory ("cubeprogrammer-{0}-attempt-{1:D2}-{2}.log" -f $Tag, $attemptNumber, $script:ActiveFrequencyKhz)
        $script:Stage = "hotplug-$Tag"
        $exitCode = $null
        $text = ''
        $previousPreference = $ErrorActionPreference
        try {
            $ErrorActionPreference = 'Continue'
            $output = @(& $script:ProgrammerPath @arguments 2>&1)
            $exitCode = $LASTEXITCODE
            $text = (($output | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine)
        }
        catch {
            $text = $_.Exception.Message
        }
        finally { $ErrorActionPreference = $previousPreference }
        Set-Content -LiteralPath $logPath -Value $text -Encoding UTF8
        $actualFrequency = $null
        if ($text -match '(?im)^\s*SWD\s+freq\s*:\s*(\d+)\s*KHz\s*$') {
            $actualFrequency = [int]$Matches[1]
        }
        $attemptErrorClass = if ($exitCode -eq 0) { $null } else {
            Resolve-ToolErrorClass -Message $text -Stage $script:Stage
        }
        $attempt = [ordered]@{
            stage = $script:Stage
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
                throw "STM32CubeProgrammer reported success but did not create $Destination."
            }
            $actualSize = (Get-Item -LiteralPath $Destination).Length
            if ($actualSize -ne $Size) {
                $script:ErrorClass = 'ram_read'
                $attempt.error_class = $script:ErrorClass
                Save-ConnectionAttempts
                throw "STM32CubeProgrammer wrote $actualSize bytes for a $Size-byte $Tag read."
            }
            if ($actualFrequency) { $script:ActualFrequencyKhz = $actualFrequency }
            $script:Connected = $true
            return
        }
        if ($attemptErrorClass -eq 'core_inaccessible' -and $index + 1 -lt $frequencies.Count) {
            continue
        }
        $script:ErrorClass = $attemptErrorClass
        throw "STM32CubeProgrammer failed during $Tag read (exit $exitCode; error_class=$attemptErrorClass). See $logPath. $text"
    }
}

function Write-FailedStatus {
    param([string]$Message, [string]$ElfPath, [string]$StackTrace)
    if (-not $script:RunDirectory) { return }
    $rawFiles = @()
    foreach ($name in @('flash-rom-descriptor.bin', 'scalar-region.bin')) {
        $path = Join-Path $script:RunDirectory $name
        if (Test-Path -LiteralPath $path -PathType Leaf) {
            $rawFiles += [ordered]@{ path = $path; bytes = (Get-Item -LiteralPath $path).Length }
        }
    }
    foreach ($pattern in @('runtime-log-header-*.bin', 'runtime-diagnostic-*.bin', 'speed-pi-diagnostic-*.bin', 'ackermann-snapshot-*.bin', 'ackermann-generation-*.bin')) {
        foreach ($file in @(Get-ChildItem -LiteralPath $script:RunDirectory -Filter $pattern -File -ErrorAction SilentlyContinue)) {
            $rawFiles += [ordered]@{ path = $file.FullName; bytes = $file.Length }
        }
    }
    $failed = [ordered]@{
        status = 'failed'
        error_class = Resolve-ToolErrorClass -Message $Message -Stage $script:Stage
        snapshot_kind = 'live_diagnostic_only'
        stage = $script:Stage
        started_at_utc = $script:StartedAtUtc
        finished_at_utc = [DateTime]::UtcNow.ToString('o')
        error = $Message
        script_stack_trace = $StackTrace
        elf = $ElfPath
        elf_sha256 = if ($script:Plan) { $script:Plan.elf_sha256 } else { $null }
        build_id = if ($script:Plan) { $script:Plan.build_id } else { $null }
        target_probe_sn = $script:SelectedProbeSN
        requested_connection_frequency_khz = $ConnectionFrequencyKhz
        connection_frequency_khz = $script:ActiveFrequencyKhz
        actual_connection_frequency_khz = $script:ActualFrequencyKhz
        shared_connection = [bool]$script:SharedConnectionValue
        connection = "SWD HOTPLUG freq=$script:ActiveFrequencyKhz$(if ($script:SharedConnectionValue) { ' shared' })$(if ($script:SelectedProbeSN) { ' sn=' + $script:SelectedProbeSN })"
        connection_attempts = $script:ConnectionAttempts.ToArray()
        probe_list_log = if (Test-Path -LiteralPath (Join-Path $script:RunDirectory 'probe-list.log')) { Join-Path $script:RunDirectory 'probe-list.log' } else { $null }
        probe_selection = if (Test-Path -LiteralPath (Join-Path $script:RunDirectory 'probe-selection.json')) { Join-Path $script:RunDirectory 'probe-selection.json' } else { $null }
        record_area_read = $false
        reads_are_atomic_together = $false
        raw_files = $rawFiles
    }
    $failed | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-status.json') -Encoding UTF8
}

function Get-OptionalAckFieldValue {
    param(
        [AllowNull()][object]$Snapshot,
        [Parameter(Mandatory = $true)][string]$FieldName
    )

    if ($null -eq $Snapshot) { return 'N/A' }
    $property = $Snapshot.PSObject.Properties[$FieldName]
    if ($null -eq $property -or $null -eq $property.Value) { return 'N/A' }
    return [string]$property.Value
}

try {
    $elfPath = $null
    $elfPath = (Resolve-Path -LiteralPath $Elf -ErrorAction Stop).Path
    $script:PythonExe = Resolve-PythonExe
    if (-not (Test-Path -LiteralPath $script:Reader -PathType Leaf)) {
        throw "Status reader helper is missing: $script:Reader"
    }

    if ($DryRun) {
        $plan = Invoke-ReaderJson -Arguments @($script:Reader, 'plan', '--elf', $elfPath) -LogPath ''
        $cliPath = if ($ProgrammerCLI) { [IO.Path]::GetFullPath($ProgrammerCLI) } else { '<auto-detected when run>' }
        $script:ActiveFrequencyKhz = $ConnectionFrequencyKhz
        $script:SelectedProbeSN = if ($ProbeSN) { $ProbeSN } else { '<unique probe from -l st-link-only>' }
        $descriptorDestination = '<diagnostic-directory>\flash-rom-descriptor.bin'
        $scalarDestination = '<diagnostic-directory>\scalar-region.bin'
        $headerBeforeDestination = '<diagnostic-directory>\runtime-log-header-before.bin'
        $headerAfterDestination = '<diagnostic-directory>\runtime-log-header-after.bin'
        $diagnosticDestination = '<diagnostic-directory>\runtime-diagnostic.bin'
        $speedPiDiagnosticDestination = '<diagnostic-directory>\speed-pi-diagnostic.bin'
        $receiveDiagnosticDestination = '<diagnostic-directory>\receive-diagnostic.bin'
        $speedPiTraceHeaderDestination = '<diagnostic-directory>\speed-pi-trace-header.bin'
        $ackermannSnapshotDestination = '<diagnostic-directory>\ackermann-snapshot.bin'
        $ackermannGenerationBeforeDestination = '<diagnostic-directory>\ackermann-generation-before.bin'
        $ackermannGenerationAfterDestination = '<diagnostic-directory>\ackermann-generation-after.bin'
        $plannedReads = New-Object System.Collections.Generic.List[object]
        $plannedReads.Add([ordered]@{ purpose = '64-byte Flash ROM descriptor identity check'; arguments = @(Get-ReadArguments -Address $plan.descriptor_read.address -Size 64 -Destination $descriptorDestination) })
        $plannedReads.Add([ordered]@{ purpose = 'firmware status and available hold-entry snapshot scalars'; arguments = @(Get-ReadArguments -Address $plan.scalar_read.address -Size $plan.scalar_read.size -Destination $scalarDestination) })
        $plannedReads.Add([ordered]@{ purpose = 'live H743 GPIOE IDR, PE7 bit 7; independent of cached firmware scalars'; arguments = @(Get-ReadArguments -Address '0x58021010' -Size 4 -Destination '<diagnostic-directory>\gpioe-idr.bin') })
        $plannedReads.Add([ordered]@{ purpose = '64-byte runtime-log header before diagnostic snapshot'; arguments = @(Get-ReadArguments -Address $plan.header_read.address -Size 64 -Destination $headerBeforeDestination) })
        if ($plan.runtime_diagnostic_snapshot.available) {
            $plannedReads.Add([ordered]@{ purpose = '64-byte generation/CRC-protected runtime diagnostic snapshot'; arguments = @(Get-ReadArguments -Address $plan.runtime_diagnostic_snapshot.address -Size 64 -Destination $diagnosticDestination) })
        }
        if ($plan.speed_pi_diagnostic_snapshot.available) {
            $plannedReads.Add([ordered]@{ purpose = '128-byte generation/CRC-protected speed PI session snapshot'; arguments = @(Get-ReadArguments -Address $plan.speed_pi_diagnostic_snapshot.address -Size 128 -Destination $speedPiDiagnosticDestination) })
        }
        if ($plan.receive_diagnostic_snapshot.available) {
            $plannedReads.Add([ordered]@{ purpose = '96-byte generation/CRC-protected receive-rate and sequence snapshot'; arguments = @(Get-ReadArguments -Address $plan.receive_diagnostic_snapshot.address -Size 96 -Destination $receiveDiagnosticDestination) })
        }
        if ($plan.speed_pi_trace_snapshot.available) {
            $plannedReads.Add([ordered]@{ purpose = '64-byte generation/CRC-protected speed PI trace header'; arguments = @(Get-ReadArguments -Address $plan.speed_pi_trace_snapshot.address -Size 64 -Destination $speedPiTraceHeaderDestination) })
        }
        if ($plan.ackermann_snapshot.available) {
            $plannedReads.Add([ordered]@{ purpose = '4-byte ACK1 generation guard before the coordination snapshot'; arguments = @(Get-ReadArguments -Address $plan.ackermann_snapshot.generation_read.address -Size 4 -Destination $ackermannGenerationBeforeDestination) })
            $plannedReads.Add([ordered]@{ purpose = '64-byte CRC32-protected ACK1 coordination snapshot'; arguments = @(Get-ReadArguments -Address $plan.ackermann_snapshot.address -Size 64 -Destination $ackermannSnapshotDestination) })
            $plannedReads.Add([ordered]@{ purpose = '4-byte ACK1 generation guard after the coordination snapshot'; arguments = @(Get-ReadArguments -Address $plan.ackermann_snapshot.generation_read.address -Size 4 -Destination $ackermannGenerationAfterDestination) })
        }
        $plannedReads.Add([ordered]@{ purpose = '64-byte runtime-log header after diagnostic snapshot'; arguments = @(Get-ReadArguments -Address $plan.header_read.address -Size 64 -Destination $headerAfterDestination) })
        $result = [ordered]@{
            status = 'dry_run'
            connected = $false
            elf = $plan.elf_path
            elf_sha256 = $plan.elf_sha256
            build_id = $plan.build_id
            log_version = $plan.log_version
            hold_quantum_ms = $plan.hold_quantum_ms
            programmer_cli = $cliPath
            probe_enumeration_arguments = @('-l', 'st-link-only')
            selected_probe_sn = if ($ProbeSN) { $ProbeSN } else { '<unique probe from -l st-link-only>' }
            frequency_attempts_khz = @(Get-ConnectionFrequencies)
            requested_connection_frequency_khz = $ConnectionFrequencyKhz
            connection_frequency_khz = $ConnectionFrequencyKhz
            shared_connection = [bool]$script:SharedConnectionValue
            reads = $plannedReads.ToArray()
            record_area_read = $false
        }
        $result | ConvertTo-Json -Depth 8
        exit 0
    }

    $script:RunDirectory = New-ReadDirectory -Root $OutDirectory
    $script:Stage = 'inspect-elf'
    $script:Plan = Invoke-ReaderJson -Arguments @($script:Reader, 'plan', '--elf', $elfPath) `
        -LogPath (Join-Path $script:RunDirectory 'python-plan.log')
    $script:Plan | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'read-plan.json') -Encoding UTF8
    $script:ProgrammerPath = Resolve-ProgrammerCli

    $script:Stage = 'enumerate-probe'
    Resolve-ProbeSerial

    $descriptorPath = Join-Path $script:RunDirectory 'flash-rom-descriptor.bin'
    Invoke-StatusRead -Tag 'flash-descriptor' -Address $script:Plan.descriptor_read.address `
        -Size $script:Plan.descriptor_read.size -Destination $descriptorPath
    $script:Stage = 'validate-flash-descriptor'
    $descriptorValidation = Invoke-ReaderJson -Arguments @(
        $script:Exporter, 'check-descriptor', '--elf', $elfPath, '--descriptor', $descriptorPath
    ) -LogPath (Join-Path $script:RunDirectory 'python-check-flash-descriptor.log')
    $script:Plan | Add-Member -NotePropertyName target_flash_descriptor -NotePropertyValue $descriptorValidation

    $scalarPath = Join-Path $script:RunDirectory 'scalar-region.bin'
    Invoke-StatusRead -Tag 'scalars' -Address $script:Plan.scalar_read.address `
        -Size $script:Plan.scalar_read.size -Destination $scalarPath
    $diagnosticAvailable = [bool]$script:Plan.runtime_diagnostic_snapshot.available
    $speedPiDiagnosticAvailable = [bool]$script:Plan.speed_pi_diagnostic_snapshot.available
    $receiveDiagnosticAvailable = [bool]$script:Plan.receive_diagnostic_snapshot.available
    $speedPiTraceAvailable = [bool]$script:Plan.speed_pi_trace_snapshot.available
    $ackermannSnapshotAvailable = [bool]$script:Plan.ackermann_snapshot.available
    $decoded = $null
    $decodeFailure = $null
    for ($attempt = 1; $attempt -le 3; ++$attempt) {
        $suffix = '{0:D2}' -f $attempt
        $headerBeforePath = Join-Path $script:RunDirectory ("runtime-log-header-before-{0}.bin" -f $suffix)
        $headerAfterPath = Join-Path $script:RunDirectory ("runtime-log-header-after-{0}.bin" -f $suffix)
        $diagnosticPath = Join-Path $script:RunDirectory ("runtime-diagnostic-{0}.bin" -f $suffix)
        $speedPiDiagnosticPath = Join-Path $script:RunDirectory ("speed-pi-diagnostic-{0}.bin" -f $suffix)
        $receiveDiagnosticPath = Join-Path $script:RunDirectory ("receive-diagnostic-{0}.bin" -f $suffix)
        $speedPiTraceHeaderPath = Join-Path $script:RunDirectory ("speed-pi-trace-header-{0}.bin" -f $suffix)
        $ackermannSnapshotPath = Join-Path $script:RunDirectory ("ackermann-snapshot-{0}.bin" -f $suffix)
        $ackermannGenerationBeforePath = Join-Path $script:RunDirectory ("ackermann-generation-before-{0}.bin" -f $suffix)
        $ackermannGenerationAfterPath = Join-Path $script:RunDirectory ("ackermann-generation-after-{0}.bin" -f $suffix)
        Invoke-StatusRead -Tag ("header-before-{0}" -f $suffix) `
            -Address $script:Plan.header_read.address -Size 64 -Destination $headerBeforePath
        if ($diagnosticAvailable) {
            Invoke-StatusRead -Tag ("runtime-diagnostic-{0}" -f $suffix) `
                -Address $script:Plan.runtime_diagnostic_snapshot.address -Size 64 -Destination $diagnosticPath
        }
        if ($speedPiDiagnosticAvailable) {
            Invoke-StatusRead -Tag ("speed-pi-diagnostic-{0}" -f $suffix) `
                -Address $script:Plan.speed_pi_diagnostic_snapshot.address -Size 128 -Destination $speedPiDiagnosticPath
        }
        if ($receiveDiagnosticAvailable) {
            Invoke-StatusRead -Tag ("receive-diagnostic-{0}" -f $suffix) `
                -Address $script:Plan.receive_diagnostic_snapshot.address -Size 96 -Destination $receiveDiagnosticPath
        }
        if ($speedPiTraceAvailable) {
            Invoke-StatusRead -Tag ("speed-pi-trace-header-{0}" -f $suffix) `
                -Address $script:Plan.speed_pi_trace_snapshot.address -Size 64 -Destination $speedPiTraceHeaderPath
        }
        if ($ackermannSnapshotAvailable) {
            Invoke-StatusRead -Tag ("ackermann-generation-before-{0}" -f $suffix) `
                -Address $script:Plan.ackermann_snapshot.generation_read.address -Size 4 -Destination $ackermannGenerationBeforePath
            Invoke-StatusRead -Tag ("ackermann-snapshot-{0}" -f $suffix) `
                -Address $script:Plan.ackermann_snapshot.address -Size 64 -Destination $ackermannSnapshotPath
            Invoke-StatusRead -Tag ("ackermann-generation-after-{0}" -f $suffix) `
                -Address $script:Plan.ackermann_snapshot.generation_read.address -Size 4 -Destination $ackermannGenerationAfterPath
        }
        Invoke-StatusRead -Tag ("header-after-{0}" -f $suffix) `
            -Address $script:Plan.header_read.address -Size 64 -Destination $headerAfterPath

        $decodeArguments = @(
            $script:Reader, 'decode', '--elf', $elfPath, '--scalars', $scalarPath,
            '--header', $headerBeforePath, '--header-after', $headerAfterPath,
            '--descriptor', $descriptorPath
        )
        if ($diagnosticAvailable) {
            $decodeArguments += @('--runtime-diagnostic', $diagnosticPath)
        }
        if ($speedPiDiagnosticAvailable) {
            $decodeArguments += @('--speed-pi-diagnostic', $speedPiDiagnosticPath)
        }
        if ($receiveDiagnosticAvailable) {
            $decodeArguments += @('--receive-diagnostic', $receiveDiagnosticPath)
        }
        if ($speedPiTraceAvailable) {
            $decodeArguments += @('--speed-pi-trace-header', $speedPiTraceHeaderPath)
        }
        if ($ackermannSnapshotAvailable) {
            $decodeArguments += @('--ackermann-snapshot', $ackermannSnapshotPath,
                '--ackermann-generation-before', $ackermannGenerationBeforePath,
                '--ackermann-generation-after', $ackermannGenerationAfterPath)
        }
        $script:Stage = "decode-snapshot-attempt-$suffix"
        try {
            $decoded = Invoke-ReaderJson -Arguments $decodeArguments `
                -LogPath (Join-Path $script:RunDirectory ("python-decode-{0}.log" -f $suffix))
            $decodeFailure = $null
        }
        catch {
            $decodeFailure = $_.Exception.Message
            if ($attempt -lt 3) {
                Add-Content -LiteralPath (Join-Path $script:RunDirectory 'snapshot-retries.log') `
                    -Value ("Attempt {0}: {1}" -f $attempt, $decodeFailure) -Encoding UTF8
                continue
            }
            throw
        }

        if ($decoded.status_class -notin @('inconsistent', 'legacy_inconsistent')) {
            break
        }
        if ($attempt -lt 3) {
            Add-Content -LiteralPath (Join-Path $script:RunDirectory 'snapshot-retries.log') `
                -Value ("Attempt {0}: {1}" -f $attempt, (@($decoded.warnings) -join '; ')) -Encoding UTF8
        }
    }
    if ($null -eq $decoded) {
        throw "No valid status snapshot was decoded after three attempts. $decodeFailure"
    }
    # H743 GPIOE base and IDR offset come from the project's CMSIS device header.
    # Legacy car_pe7_raw_high is assigned during initialization only: it is not
    # a live pin sample. Keep it for compatibility, but read the peripheral too.
    $gpioeIdrPath = Join-Path $script:RunDirectory 'gpioe-idr.bin'
    Invoke-StatusRead -Tag 'gpioe-idr' -Address '0x58021010' -Size 4 -Destination $gpioeIdrPath
    $gpioeIdrBytes = [IO.File]::ReadAllBytes($gpioeIdrPath)
    if ($gpioeIdrBytes.Length -ne 4) { throw 'GPIOE IDR read must contain exactly four bytes.' }
    $gpioeIdrValue = [BitConverter]::ToUInt32($gpioeIdrBytes, 0)
    $decoded | Add-Member -NotePropertyName pe7_gpio_snapshot -NotePropertyValue ([ordered]@{
        source = 'H743 GPIOE->IDR bit 7, read-only HOTPLUG'
        address = '0x58021010'
        idr_u32 = $gpioeIdrValue
        raw_high = [int](($gpioeIdrValue -shr 7) -band 1)
        sampled_at_utc = [DateTime]::UtcNow.ToString('o')
        raw_file = $gpioeIdrPath
        atomic_with_firmware_snapshot = $false
    })
    $decoded | Add-Member -NotePropertyName generated_at_utc -NotePropertyValue ([DateTime]::UtcNow.ToString('o'))
    $decoded | Add-Member -NotePropertyName connection -NotePropertyValue ([ordered]@{
        probe_sn = $script:SelectedProbeSN
        requested_frequency_khz = $ConnectionFrequencyKhz
        frequency_khz = $script:ActiveFrequencyKhz
        actual_frequency_khz = $script:ActualFrequencyKhz
        shared = [bool]$script:SharedConnectionValue
        attempts = $script:ConnectionAttempts.ToArray()
    })
    $decoded | Add-Member -NotePropertyName probe_list_log -NotePropertyValue (Join-Path $script:RunDirectory 'probe-list.log')
    $decoded | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-status.json') -Encoding UTF8

    $state = $decoded.runtime_log_header_after.state_name
    $count = $decoded.runtime_log_header_after.count
    $mode = $decoded.scalars.car_mode_name
    $fault = $decoded.scalars.car_fault_name
    $version = $decoded.elf.log_version
    $enabled = if ($decoded.runtime_diagnostic_snapshot) { $decoded.runtime_diagnostic_snapshot.log_enabled } else { $decoded.scalars.car_runtime_log_enabled }
    $starts = if ($decoded.runtime_diagnostic_snapshot) { $decoded.runtime_diagnostic_snapshot.start_events } else { $decoded.scalars.car_button_start_event_count }
    $heartbeat = $decoded.scalars.car_main_loop_heartbeat_ms
    $resetFlag = $decoded.scalars.iwdg_reset_flag_in_rcc_rsr
    $snapshot = $decoded.scalars.car_vision_hold_snapshot_published
    $diagState = if ($decoded.runtime_diagnostic_snapshot) { $decoded.runtime_diagnostic_snapshot.cpu_header_state_name } else { 'unavailable-legacy-ELF' }
    $diagCount = if ($decoded.runtime_diagnostic_snapshot) { $decoded.runtime_diagnostic_snapshot.cpu_header_count } else { $null }
    $startToken = if ($decoded.runtime_diagnostic_snapshot) { $decoded.runtime_diagnostic_snapshot.start_token } else { $null }
    $ack = $decoded.ackermann_snapshot
    $ackStatus = if ($ack) { $ack.status } else { 'unavailable-legacy-ELF' }
    $ackError = if ($ack -and $ack.error_class) { $ack.error_class } else { 'none' }
    Write-Output ("Status: {0}; logVersion={1}; CPU logger={2}, header={3}, enabled={4}, CPU count={5}, header count={6}, start events={7}, start token={8}; heartbeat_ms={9}; mode={10}, fault={11}, iwdgReset={12}; holdSnapshot={13}; activeProgress={14}; ACK1={15}/{16}" -f $decoded.status_class, $version, $diagState, $state, $enabled, $diagCount, $count, $starts, $startToken, $heartbeat, $mode, $fault, $resetFlag, $snapshot, $decoded.observing_active_progress, $ackStatus, $ackError)
    Write-Output ("Target probe={0}; requested SWD={1} kHz; requested SWD rate reported by probe={2} kHz; shared={3}; CSV created={4}" -f $script:SelectedProbeSN, $ConnectionFrequencyKhz, $script:ActualFrequencyKhz, [bool]$script:SharedConnectionValue, $decoded.csv_created)
    if ($decoded.scalars.PSObject.Properties.Name -contains 'car_button_state_name') {
        $pe7Status = $decoded.scalars
        Write-Output ("PE7: GPIO_high={0} (live IDR); firmware_cached_high={1}; FSM={2}; RUN_request={3}; start_pending={4}; starts={5}; stops={6}; 1ms_samples={7}; sample_max_gap_ms={8}" -f $decoded.pe7_gpio_snapshot.raw_high, $pe7Status.car_pe7_raw_high, $pe7Status.car_button_state_name, $pe7Status.car_manual_run_request, $pe7Status.car_start_pending, $pe7Status.car_button_start_event_count, $pe7Status.car_button_stop_generation, $pe7Status.car_pe7_sample_count, $pe7Status.car_pe7_sample_gap_max_ms)
        Write-Output ("Motor gates: block_reason={0}; drive_allowed={1}; hardware_lock={2}; servo_first_frame={3}; startup_qualified={4}; PWM_enabled={5}; TIM3_CCR2={6}; TIM3_CCR3={7}; TIM3_CCER={8}" -f $pe7Status.car_run_block_reason, $pe7Status.car_drive_allowed, $pe7Status.car_motor_hardware_lock_reason, $pe7Status.car_servo_frame_ready, $pe7Status.car_follow_startup_qualified, $pe7Status.car_motor_pwm_enabled, $pe7Status.car_motor_pwm_ccr2, $pe7Status.car_motor_pwm_ccr3, $pe7Status.car_motor_pwm_ccer)
    }
    if ($decoded.status_class -in @('neverstarted', 'legacy_idle')) { Write-Output 'IDLE: no run records are available; this status tool does not create a CSV.' }
    if ($decoded.status_class -in @('active', 'legacy_active')) { Write-Output 'ACTIVE: the logger is still recording; wait for the vehicle stop path to freeze the log before export.' }
    if ($decoded.status_class -eq 'frozen') { Write-Output 'FROZEN: CPU snapshot and stable runtime-log header agree on state, count, and session.' }
    if ($decoded.status_class -eq 'inconsistent' -or $decoded.status_class -eq 'legacy_inconsistent') { Write-Output 'INCONSISTENT: the bounded header/snapshot comparison did not settle; retain diagnostics and do not export.' }
    if ($decoded.runtime_diagnostic_snapshot -and $decoded.runtime_diagnostic_snapshot.start_events -gt 0 -and $decoded.runtime_diagnostic_snapshot.start_token -eq 0) { Write-Output 'PE7 start events were seen, but the logger has not consumed a start token.' }
    if ($decoded.runtime_diagnostic_snapshot -and -not $decoded.runtime_diagnostic_snapshot.log_enabled) { Write-Output 'Runtime logger is disabled in this firmware build.' }
    if ($decoded.PSObject.Properties.Name -contains 'camera_uart_diagnostics') {
        $camera = $decoded.camera_uart_diagnostics
        if ($camera.available) {
            Write-Output ("Camera UART diagnostics: status={0}; fresh={1}; source=camera records cached by H743 (not full text logs)" -f $camera.status, $camera.fresh)
            if ($camera.PSObject.Properties.Name -contains 'error') {
                Write-Output ("Camera diagnostic validation: {0}" -f $camera.error)
            }
            if ($camera.PSObject.Properties.Name -contains 'latest_status' -and $camera.latest_status) {
                $cs = $camera.latest_status
                Write-Output ("Camera status: age_ms={0}; map={1}; parked_check={2}; gate_blocked={3}; observation={4}; reverse={5}; profile_raw={6}; ACK_actually_sent={7}; valid_actually_sent={8}" -f $cs.age_ms, $cs.calibration_reason, $cs.parked_check_passed, $cs.calibration_gate_blocked, $cs.observation_reason, $cs.reverse_reason, $cs.raw_profile_ack, $cs.actual_transmitted_ack, $cs.actual_observation_valid)
            }
            if ($camera.PSObject.Properties.Name -contains 'latest_counters' -and $camera.latest_counters) {
                $cc = $camera.latest_counters
                Write-Output ("Camera counters: age_ms={0}; vision_tx_errors={1}; detector_errors={2}; reverse_rejects={3}; saturated={4}" -f $cc.age_ms, $cc.vision_tx_errors, $cc.detector_errors, $cc.reverse_rejects, $cc.saturated)
            }
            if ($camera.PSObject.Properties.Name -contains 'latest_identity' -and $camera.latest_identity) {
                Write-Output ("Camera diagnostic build marker: {0}; this marker does not verify the complete camera source bundle." -f $camera.latest_identity.build_code)
            }
            foreach ($record in @($camera.records)) {
                if ($record.kind -eq 'status') {
                    Write-Output ("Camera record seq={0} at_h743_ms={1} age_ms={2}: map={3}, observation={4}, ACK={5}->{6}, valid={7}" -f $record.diagnostic_sequence, $record.arrival_ms, $record.age_ms, $record.calibration_reason, $record.observation_reason, $record.raw_profile_ack, $record.actual_transmitted_ack, $record.actual_observation_valid)
                } elseif ($record.kind -eq 'counters') {
                    Write-Output ("Camera record seq={0} at_h743_ms={1} age_ms={2}: txerr={3}, deterr={4}, reverse_rejects={5}" -f $record.diagnostic_sequence, $record.arrival_ms, $record.age_ms, $record.vision_tx_errors, $record.detector_errors, $record.reverse_rejects)
                } elseif ($record.kind -eq 'identity') {
                    Write-Output ("Camera record seq={0} at_h743_ms={1} age_ms={2}: diagnostic_build={3}" -f $record.diagnostic_sequence, $record.arrival_ms, $record.age_ms, $record.build_code)
                }
            }
        } else {
            Write-Output 'Camera UART diagnostics: unsupported by selected ELF; camera state is unavailable through this snapshot.'
        }
    }
    if ($decoded.speed_pi_diagnostic_snapshot) {
        $pi = $decoded.speed_pi_diagnostic_snapshot
        Write-Output ("Speed PI: type={0}; status={1}; token={2} (matches runtime token={3}); target_mRPS={4}/{5}; measured_mRPS={6}/{7}; nominal_FF={8}/{9}; requested={10}/{11}; applied={12}/{13}; upper-cap_ms={14}/{15}; active={16}; frozen={17}; hold={18}" -f $pi.pi_type_name, $pi.status_name, $pi.start_token, $pi.start_token_matches_runtime_diagnostic, $pi.target_left_millirps, $pi.target_right_millirps, $pi.measured_left_millirps, $pi.measured_right_millirps, $pi.nominal_ff_left_permille, $pi.nominal_ff_right_permille, $pi.requested_left_permille, $pi.requested_right_permille, $pi.applied_left_permille, $pi.applied_right_permille, $pi.saturation_left_ms, $pi.saturation_right_ms, $pi.active, $pi.frozen, $pi.hold_active)
    }
    if ($decoded.receive_diagnostic_snapshot) {
        $rx = $decoded.receive_diagnostic_snapshot
        $rxRate = if ($null -eq $rx.accepted_fps) { 'warming' } else { '{0:F3}' -f $rx.accepted_fps }
        Write-Output ("RX window: accepted={0}/{1} ms ({2} parser-accepted fps); invalid-observation={3}; missing-seq={4}; rejected-seq={5}; parser-errors={6}; latest-seq={7}" -f $rx.accepted_window, $rx.window_duration_ms, $rxRate, $rx.invalid_observation_window, $rx.missing_sequence_window, $rx.sequence_reject_window, $rx.parser_error_total, $rx.latest_sequence)
    }
    if ($decoded.speed_pi_trace_header) {
        $trace = $decoded.speed_pi_trace_header
        Write-Output ("PI trace: state={0}; type={1}; token={2}; count={3}; total={4}; lost={5}; build-id={6}" -f $trace.state_name, $trace.pi_type_name, $trace.start_token, $trace.count, $trace.total_samples, $trace.lost_samples, $trace.build_id)
    }
    if ($decoded.ackermann_snapshot) {
        $ack = $decoded.ackermann_snapshot
        $ackStatus = Get-OptionalAckFieldValue -Snapshot $ack -FieldName 'status'
        $ackValid = Get-OptionalAckFieldValue -Snapshot $ack -FieldName 'valid'
        $ackExportable = Get-OptionalAckFieldValue -Snapshot $ack -FieldName 'exportable'
        $ackGeneration = Get-OptionalAckFieldValue -Snapshot $ack -FieldName 'generation'
        $ackPhase = Get-OptionalAckFieldValue -Snapshot $ack -FieldName 'curve_phase_name'
        $ackSlowdown = Get-OptionalAckFieldValue -Snapshot $ack -FieldName 'slowdown_request_permille'
        $ackFrozen = Get-OptionalAckFieldValue -Snapshot $ack -FieldName 'frozen'
        $ackSessionActive = Get-OptionalAckFieldValue -Snapshot $ack -FieldName 'session_active'
        Write-Output ("ACK1 snapshot: status={0}; valid={1}; exportable={2}; generation={3}; phase={4}; slowdown_request_permille={5}; frozen={6}; session_active={7}" -f $ackStatus, $ackValid, $ackExportable, $ackGeneration, $ackPhase, $ackSlowdown, $ackFrozen, $ackSessionActive)
    }
    foreach ($warning in @($decoded.warnings)) { Write-Output ("Warning: {0}" -f $warning) }
    Write-Output ("Diagnostic JSON: {0}" -f (Join-Path $script:RunDirectory 'run-status.json'))
    Write-Output ("Raw snapshot files retained in: {0}" -f $script:RunDirectory)
    if ($decoded.status_class -in @('inconsistent', 'legacy_inconsistent')) { exit 2 }
    exit 0
}
catch {
    $errorRecord = $_
    $message = $errorRecord.Exception.Message
    if (-not $script:RunDirectory -and -not $DryRun) {
        try { $script:RunDirectory = New-ReadDirectory -Root $OutDirectory } catch { }
    }
    if (-not $elfPath) { $elfPath = [IO.Path]::GetFullPath($Elf) }
    Write-FailedStatus -Message $message -ElfPath $elfPath -StackTrace $errorRecord.ScriptStackTrace
    Write-Error ("Run-status read failed at stage '{0}'. {1}" -f $script:Stage, $message)
    exit 1
}
