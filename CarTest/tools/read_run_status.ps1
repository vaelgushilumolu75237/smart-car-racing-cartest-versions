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
    foreach ($name in @('flash-rom-descriptor.bin', 'scalar-region.bin', 'runtime-log-header.bin')) {
        $path = Join-Path $script:RunDirectory $name
        if (Test-Path -LiteralPath $path -PathType Leaf) {
            $rawFiles += [ordered]@{ path = $path; bytes = (Get-Item -LiteralPath $path).Length }
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
        $headerDestination = '<diagnostic-directory>\runtime-log-header.bin'
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
            reads = @(
                [ordered]@{ purpose = '64-byte Flash ROM descriptor identity check'; arguments = @(Get-ReadArguments -Address $plan.descriptor_read.address -Size 64 -Destination $descriptorDestination) },
                [ordered]@{ purpose = 'firmware status and available hold-entry snapshot scalars'; arguments = @(Get-ReadArguments -Address $plan.scalar_read.address -Size $plan.scalar_read.size -Destination $scalarDestination) },
                [ordered]@{ purpose = '64-byte runtime-log header'; arguments = @(Get-ReadArguments -Address $plan.header_read.address -Size $plan.header_read.size -Destination $headerDestination) }
            )
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
    $headerPath = Join-Path $script:RunDirectory 'runtime-log-header.bin'
    Invoke-StatusRead -Tag 'scalars' -Address $script:Plan.scalar_read.address `
        -Size $script:Plan.scalar_read.size -Destination $scalarPath
    Invoke-StatusRead -Tag 'header' -Address $script:Plan.header_read.address `
        -Size $script:Plan.header_read.size -Destination $headerPath

    $script:Stage = 'decode-snapshot'
    $decoded = Invoke-ReaderJson -Arguments @(
        $script:Reader, 'decode', '--elf', $elfPath, '--scalars', $scalarPath,
        '--header', $headerPath, '--descriptor', $descriptorPath
    ) -LogPath (Join-Path $script:RunDirectory 'python-decode.log')
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

    $state = $decoded.runtime_log_header.state_name
    $count = $decoded.runtime_log_header.count
    $mode = $decoded.scalars.car_mode_name
    $fault = $decoded.scalars.car_fault_name
    $version = $decoded.elf.log_version
    $enabled = $decoded.scalars.car_runtime_log_enabled
    $starts = $decoded.scalars.car_button_start_event_count
    $heartbeat = $decoded.scalars.car_main_loop_heartbeat_ms
    $resetFlag = $decoded.scalars.iwdg_reset_flag_in_rcc_rsr
    $snapshot = $decoded.scalars.car_vision_hold_snapshot_published
    Write-Output ("Status: {0}; logVersion={1}; logger={2}, enabled={3}, count={4}, starts={5}; heartbeat_ms={6}; mode={7}, fault={8}, iwdgReset={9}; holdSnapshot={10}" -f $decoded.status, $version, $state, $enabled, $count, $starts, $heartbeat, $mode, $fault, $resetFlag, $snapshot)
    Write-Output ("Target probe={0}; requested SWD={1} kHz; requested SWD rate reported by probe={2} kHz; shared={3}; CSV created={4}" -f $script:SelectedProbeSN, $ConnectionFrequencyKhz, $script:ActualFrequencyKhz, [bool]$script:SharedConnectionValue, $decoded.csv_created)
    if ($state -eq 'idle') { Write-Output 'IDLE: count=0; no run records are available and this status tool does not create a CSV.' }
    foreach ($warning in @($decoded.warnings)) { Write-Output ("Warning: {0}" -f $warning) }
    Write-Output ("Diagnostic JSON: {0}" -f (Join-Path $script:RunDirectory 'run-status.json'))
    Write-Output ("Raw snapshot files retained in: {0}" -f $script:RunDirectory)
    if ($decoded.status -notin @('success', 'idle', 'read_with_warnings')) { exit 2 }
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
