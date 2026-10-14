[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)]
    [string]$Elf,

    [string]$OutDirectory,

    [string]$ProgrammerCLI,

    [ValidatePattern('^[A-Za-z0-9_-]{1,64}$')]
    [string]$ProbeSN,

    [ValidateSet('1000', '4000')]
    [int]$ConnectionFrequencyKhz = 4000,

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
$script:CliLogs = New-Object System.Collections.Generic.List[string]
$script:StartedAtUtc = [DateTime]::UtcNow.ToString('o')

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
    $arguments = @('-c', 'port=SWD', 'mode=HOTPLUG', "freq=$ConnectionFrequencyKhz")
    if ($SharedConnection) {
        $arguments += 'shared'
    }
    if ($ProbeSN) {
        $arguments += "sn=$ProbeSN"
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

function Invoke-HotPlugUpload {
    param(
        [Parameter(Mandatory = $true)][string]$Tag,
        [Parameter(Mandatory = $true)][string]$Address,
        [Parameter(Mandatory = $true)][long]$Size,
        [Parameter(Mandatory = $true)][string]$Destination
    )
    $arguments = New-HotPlugArguments -Address $Address -Size $Size -Destination $Destination
    $logPath = Join-Path $script:RunDirectory ("cubeprogrammer-{0}.log" -f $Tag)
    $script:CliLogs.Add($logPath)
    $script:Stage = "hotplug-$Tag"
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = @(& $script:ProgrammerPath @arguments 2>&1)
        $exitCode = $LASTEXITCODE
        $text = (($output | ForEach-Object { $_.ToString() }) -join [Environment]::NewLine)
    }
    catch {
        $text = $_.Exception.Message
        Set-Content -LiteralPath $logPath -Value $text -Encoding UTF8
        throw "Could not execute STM32CubeProgrammer during $Tag upload. See $logPath. $text"
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }
    $logLines = foreach ($line in $output) {
        if ($line -is [System.Management.Automation.ErrorRecord]) {
            "stderr: {0}" -f $line.ToString()
        }
        else {
            "stdout: {0}" -f $line.ToString()
        }
    }
    Set-Content -LiteralPath $logPath -Value ($logLines -join [Environment]::NewLine) -Encoding UTF8
    if ($exitCode -ne 0) {
        $hint = if ($text -match 'DEV_CONNECT_ERR') {
            ' DEV_CONNECT_ERR means no usable RAM header or payload was read. Keep vehicle power on; verify ProbeSN and SWD wiring, and end any debugger/IDE connection occupying the probe before retrying. Disconnecting a debugger does not stop the vehicle. Before export, use PE7 stop, verify the logger is FROZEN state=2, and keep power applied.'
        } else { '' }
        throw "STM32CubeProgrammer failed during $Tag upload (exit $exitCode). See $logPath.$hint $text"
    }
    if (-not (Test-Path -LiteralPath $Destination -PathType Leaf)) {
        throw "STM32CubeProgrammer reported success but did not create $Destination. See $logPath."
    }
    $actualSize = (Get-Item -LiteralPath $Destination).Length
    if ($actualSize -ne $Size) {
        throw "STM32CubeProgrammer wrote $actualSize bytes for a $Size-byte $Tag upload. See $logPath."
    }
}

function Get-RunMetadata {
    param([string]$Status, [string]$ErrorMessage, [object]$Layout, [object]$Header)
    $rawFiles = @()
    if ($script:RunDirectory -and (Test-Path -LiteralPath $script:RunDirectory -PathType Container)) {
        foreach ($name in @('header-initial.bin', 'runtime-log.bin', 'header-final.bin')) {
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
        elf = if ($Layout) { $Layout.elf_path } else { [IO.Path]::GetFullPath($Elf) }
        elf_sha256 = if ($Layout) { $Layout.elf_sha256 } else { $null }
        storage_address = if ($Layout) { $Layout.storage_address } else { $null }
        storage_size = if ($Layout) { $Layout.storage_size } else { $null }
        build_id = if ($Layout) { $Layout.build_id } else { $null }
        probe_serial_supplied = [bool]$ProbeSN
        connection_frequency_khz = $ConnectionFrequencyKhz
        shared_connection = [bool]$SharedConnection
        connection = "SWD HOTPLUG freq=$ConnectionFrequencyKhz$(if ($SharedConnection) { ' shared' })"
        initial_header = $Header
        record_count = if ($Header) { [int]$Header.count } else { $null }
        dropped_count = if ($Header) { [int]$Header.dropped_count } else { $null }
        buffer_full = if ($Header) { [bool]$Header.buffer_full } else { $null }
        records_truncated = if ($Header) { [bool]$Header.records_truncated } else { $null }
        csv = if ($Status -eq 'success' -and $script:RunCsvPath -and
                  (Test-Path -LiteralPath $script:RunCsvPath -PathType Leaf)) {
            $script:RunCsvPath
        } else { $null }
        planned_csv_path = $script:RunCsvPath
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
        $sampleArgs = New-HotPlugArguments -Address $headerAddress -Size 64 -Destination '<run-directory>\header-initial.bin'
        $secondRead = @(Get-HotPlugConnectionArguments) + @('--upload', $storageAddress, '<64 + frozen_count * 64>', '<run-directory>\runtime-log.bin')
        $finalRead = @(Get-HotPlugConnectionArguments) + @('--upload', $storageAddress, '64', '<run-directory>\header-final.bin')
        $plan = [ordered]@{
            dry_run = $true
            selected_elf = $layout.elf_path
            elf_sha256 = $layout.elf_sha256
            build_id = $layout.build_id
            programmer_cli = $cliPath
            connection_frequency_khz = $ConnectionFrequencyKhz
            shared_connection = [bool]$SharedConnection
            probe_serial_supplied = [bool]$ProbeSN
            ram_address = $storageAddress
            ram_size = $storageSize
            first_read = $sampleArgs
            second_read = $secondRead
            final_read = $finalRead
            output_directory = $script:RunDirectory
        }
        $plan | ConvertTo-Json -Depth 5
        return
    }

    $script:ProgrammerPath = Resolve-ProgrammerCli
    $probeArguments = New-HotPlugArguments -Address $headerAddress -Size 64 -Destination 'x'
    $forbiddenArguments = @('mode=NORMAL', '-rst', '-halt', '--download', '--erase', '-go', '--run')
    foreach ($argument in $probeArguments) {
        if ($forbiddenArguments -contains $argument) {
            throw 'Internal guard rejected a command outside the Hot Plug upload allowlist.'
        }
    }

    $initialHeaderPath = Join-Path $script:RunDirectory 'header-initial.bin'
    $script:Stage = 'read-initial-header'
    Invoke-HotPlugUpload -Tag 'header-initial' -Address $headerAddress -Size 64 -Destination $initialHeaderPath
    $script:Stage = 'validate-initial-header'
    $initialHeader = Invoke-PythonJson -Arguments @($validator, 'check-header', '--elf', $elfPath, '--header', $initialHeaderPath) -LogPath (Join-Path $script:RunDirectory 'python-check-initial-header.log')

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
    $decodeMetadata = Get-Content -LiteralPath $script:DecodeMetadataPath -Raw -Encoding UTF8 | ConvertFrom-Json
    $script:Stage = 'complete'
    $metadata = Get-RunMetadata -Status 'success' -ErrorMessage $null -Layout $layout -Header $initialHeader
    $metadata['record_count'] = [int]$decodeMetadata.record_count
    $metadata['dropped_count'] = [int]$decodeMetadata.dropped_count
    $metadata['buffer_full'] = [bool]$decodeMetadata.buffer_full
    $metadata['records_truncated'] = [bool]$decodeMetadata.records_truncated
    $metadata['raw_sha256'] = (Get-FileHash -LiteralPath $rawPath -Algorithm SHA256).Hash.ToLowerInvariant()
    $metadata | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-metadata.json') -Encoding UTF8
    Write-Output $script:RunCsvPath
    exit 0
}
catch {
    $errorMessage = $_.Exception.Message
    if ($script:RunCsvPath -and (Test-Path -LiteralPath $script:RunCsvPath -PathType Leaf)) {
        Remove-Item -LiteralPath $script:RunCsvPath -Force
    }
    if ($script:DecodeMetadataPath -and (Test-Path -LiteralPath $script:DecodeMetadataPath -PathType Leaf)) {
        Remove-Item -LiteralPath $script:DecodeMetadataPath -Force
    }
    if ($script:RunDirectory -and (Test-Path -LiteralPath $script:RunDirectory -PathType Container)) {
        try {
            $metadata = Get-RunMetadata -Status 'failed' -ErrorMessage $errorMessage -Layout $layout -Header $initialHeader
            $metadata | ConvertTo-Json -Depth 8 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-metadata.json') -Encoding UTF8
        }
        catch {
            # Preserve the primary export error if writing failure metadata itself is impossible.
        }
    }
    [Console]::Error.WriteLine($errorMessage)
    exit 1
}
