[CmdletBinding()]
param(
    [Parameter(Mandatory = $true)][string]$Elf,
    [string]$OutDirectory,
    [string]$ProgrammerCLI,
    [ValidatePattern('^[A-Za-z0-9_-]{1,64}$')][string]$ProbeSN,
    [string]$Python = 'python.exe',
    [switch]$DryRun
)

Set-StrictMode -Version Latest
$ErrorActionPreference = 'Stop'

$script:ProjectDirectory = Split-Path -Parent $PSScriptRoot
$script:Reader = Join-Path $PSScriptRoot 'run_status_reader.py'
$script:PythonExe = $null
$script:ProgrammerPath = $null
$script:RunDirectory = $null
$script:Stage = 'preflight'
$script:Plan = $null
$script:StartedAtUtc = [DateTime]::UtcNow.ToString('o')

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
    $arguments = @('-c', 'port=SWD', 'mode=HOTPLUG', 'freq=1000', 'shared')
    if ($ProbeSN) { $arguments += "sn=$ProbeSN" }
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
    $arguments = Get-ReadArguments -Address $Address -Size $Size -Destination $Destination
    $logPath = Join-Path $script:RunDirectory ("cubeprogrammer-{0}.log" -f $Tag)
    $script:Stage = "hotplug-$Tag"
    $previousPreference = $ErrorActionPreference
    try {
        $ErrorActionPreference = 'Continue'
        $output = @(& $script:ProgrammerPath @arguments 2>&1)
        $exitCode = $LASTEXITCODE
    }
    catch {
        $message = $_.Exception.Message
        Set-Content -LiteralPath $logPath -Value $message -Encoding UTF8
        throw "Could not execute STM32CubeProgrammer during $Tag read. $message"
    }
    finally {
        $ErrorActionPreference = $previousPreference
    }
    $logLines = foreach ($line in $output) {
        if ($line -is [System.Management.Automation.ErrorRecord]) {
            "stderr: {0}" -f $line.ToString()
        } else {
            "stdout: {0}" -f $line.ToString()
        }
    }
    Set-Content -LiteralPath $logPath -Value ($logLines -join [Environment]::NewLine) -Encoding UTF8
    if ($exitCode -ne 0) {
        throw "STM32CubeProgrammer failed during $Tag read (exit $exitCode). $($logLines -join ' ')"
    }
    if (-not (Test-Path -LiteralPath $Destination -PathType Leaf)) {
        throw "STM32CubeProgrammer reported success but did not create $Destination."
    }
    $actualSize = (Get-Item -LiteralPath $Destination).Length
    if ($actualSize -ne $Size) {
        throw "STM32CubeProgrammer wrote $actualSize bytes for a $Size-byte $Tag read."
    }
}

function Write-FailedStatus {
    param([string]$Message, [string]$ElfPath, [string]$StackTrace)
    if (-not $script:RunDirectory) { return }
    $rawFiles = @()
    foreach ($name in @('scalar-region.bin', 'runtime-log-header.bin')) {
        $path = Join-Path $script:RunDirectory $name
        if (Test-Path -LiteralPath $path -PathType Leaf) {
            $rawFiles += [ordered]@{ path = $path; bytes = (Get-Item -LiteralPath $path).Length }
        }
    }
    $failed = [ordered]@{
        status = 'failed'
        snapshot_kind = 'live_diagnostic_only'
        stage = $script:Stage
        started_at_utc = $script:StartedAtUtc
        finished_at_utc = [DateTime]::UtcNow.ToString('o')
        error = $Message
        script_stack_trace = $StackTrace
        elf = $ElfPath
        elf_sha256 = if ($script:Plan) { $script:Plan.elf_sha256 } else { $null }
        build_id = if ($script:Plan) { $script:Plan.build_id } else { $null }
        connection = 'SWD HOTPLUG freq=1000 shared'
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
        $scalarDestination = '<diagnostic-directory>\scalar-region.bin'
        $headerDestination = '<diagnostic-directory>\runtime-log-header.bin'
        $result = [ordered]@{
            status = 'dry_run'
            connected = $false
            elf = $plan.elf_path
            elf_sha256 = $plan.elf_sha256
            build_id = $plan.build_id
            programmer_cli = $cliPath
            reads = @(
                [ordered]@{ purpose = 'eight firmware status scalars'; arguments = @(Get-ReadArguments -Address $plan.scalar_read.address -Size $plan.scalar_read.size -Destination $scalarDestination) },
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

    $scalarPath = Join-Path $script:RunDirectory 'scalar-region.bin'
    $headerPath = Join-Path $script:RunDirectory 'runtime-log-header.bin'
    Invoke-StatusRead -Tag 'scalars' -Address $script:Plan.scalar_read.address `
        -Size $script:Plan.scalar_read.size -Destination $scalarPath
    Invoke-StatusRead -Tag 'header' -Address $script:Plan.header_read.address `
        -Size $script:Plan.header_read.size -Destination $headerPath

    $script:Stage = 'decode-snapshot'
    $decoded = Invoke-ReaderJson -Arguments @(
        $script:Reader, 'decode', '--elf', $elfPath, '--scalars', $scalarPath, '--header', $headerPath
    ) -LogPath (Join-Path $script:RunDirectory 'python-decode.log')
    $decoded | Add-Member -NotePropertyName generated_at_utc -NotePropertyValue ([DateTime]::UtcNow.ToString('o'))
    $decoded | ConvertTo-Json -Depth 12 | Set-Content -LiteralPath (Join-Path $script:RunDirectory 'run-status.json') -Encoding UTF8

    $state = $decoded.runtime_log_header.state_name
    $count = $decoded.runtime_log_header.count
    $mode = $decoded.scalars.car_mode_name
    $fault = $decoded.scalars.car_fault_name
    Write-Output ("Status: {0}; logger={1}, count={2}; mode={3}, fault={4}" -f $decoded.status, $state, $count, $mode, $fault)
    Write-Output ("Diagnostic JSON: {0}" -f (Join-Path $script:RunDirectory 'run-status.json'))
    Write-Output ("Raw snapshot files retained in: {0}" -f $script:RunDirectory)
    if ($decoded.status -ne 'success') { exit 2 }
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
